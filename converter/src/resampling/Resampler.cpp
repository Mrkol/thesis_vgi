#include "Resampler.hpp"


#define VMA_IMPLEMENTATION
#include <vk_mem_alloc.h>
#include <fstream>
#include <Eigen/Dense>

#include "../DataTypes.hpp"
#include "VkHelpers.hpp"


struct __attribute__((packed)) Vertex
{
    Eigen::Vector2f mapped_position;
    Eigen::Vector3f position;
    Eigen::Vector3f normal;
    Eigen::Vector2f uv;
};

static_assert(sizeof(Vertex) == 10*sizeof(float));

struct UniformBufferObject
{
    float frequency;
};

constexpr vk::VertexInputBindingDescription vertex_input_binding_description
    {0, sizeof(Vertex), vk::VertexInputRate::eVertex};

constexpr std::array vertex_input_attribute_descriptions{
    vk::VertexInputAttributeDescription
        {/* location */ 0, /* binding */ 0, vk::Format::eR32G32Sfloat, offsetof(Vertex, mapped_position)},
    vk::VertexInputAttributeDescription
        {/* location */ 1, /* binding */ 0, vk::Format::eR32G32B32Sfloat, offsetof(Vertex, position)},
    vk::VertexInputAttributeDescription
        {/* location */ 2, /* binding */ 0, vk::Format::eR32G32B32Sfloat, offsetof(Vertex, normal)},
    vk::VertexInputAttributeDescription
        {/* location */ 3, /* binding */ 0, vk::Format::eR32G32Sfloat, offsetof(Vertex, uv)},
};

Resampler::Resampler(const ResamplerConfig& config)
{
    resampling_resolution = (1 << uint32_t(config.log_resolution)) + 1;

    VkHelpers::check_validation_layers_support(VALIDATION_LAYERS);

    vk::ApplicationInfo app_info{"VGI Resampler", 1, "Vulkan.hpp", 1, VK_API_VERSION_1_2};
    vk::InstanceCreateInfo create_info{{}, &app_info, uint32_t(VALIDATION_LAYERS.size()), VALIDATION_LAYERS.data()};
    vk_instance = vk::createInstanceUnique(create_info);

    physical_device = vk_instance->enumeratePhysicalDevices().front();

    auto queues = physical_device.getQueueFamilyProperties();

    std::size_t queue_idx = 0;
    for (const auto& queue : queues)
    {
        if (queue.queueFlags & vk::QueueFlagBits::eGraphics && queue.queueCount >= config.thread_count)
        {
            break;
        }
        ++queue_idx;
    }

    if (queue_idx == queues.size())
    {
        throw std::runtime_error("No suitable logic device found!");
    }

    std::vector<float> priorities(config.thread_count, 0.0);
    vk::DeviceQueueCreateInfo device_queue_create_info{
        {}, uint32_t(queue_idx), uint32_t(config.thread_count), priorities.data()};
    vk::PhysicalDeviceFeatures device_features{};
    vk::DeviceCreateInfo device_create_info{{}, 1, &device_queue_create_info,
        uint32_t(VALIDATION_LAYERS.size()), VALIDATION_LAYERS.data(), {}, {}, &device_features};

    device = physical_device.createDeviceUnique(device_create_info);

    per_thread_datum.resize(config.thread_count);

    for (std::size_t idx = 0; idx < config.thread_count; ++idx)
    {
        per_thread_datum[idx].queue = device->getQueue(uint32_t(queue_idx), uint32_t(idx));
    }


    command_pool = device->createCommandPoolUnique(vk::CommandPoolCreateInfo{{}, uint32_t(queue_idx)});

    build_pipeline();

    for (auto& data : per_thread_datum)
    {
        std::array<vk::ImageView, ATTACHMENT_COUNT> views;

        for (std::size_t i = 0; i < data.image_views.size(); ++i)
        {
            data.image_staging[i] = UniqueVmaBuffer(allocator,
                resampling_resolution * resampling_resolution * OUTPUT_PIXEL_SIZE,
                vk::BufferUsageFlagBits::eTransferDst, VMA_MEMORY_USAGE_GPU_TO_CPU);

            data.images[i] = UniqueVmaImage(allocator, OUTPUT_FORMAT, {resampling_resolution, resampling_resolution},
                vk::ImageTiling::eOptimal,
                vk::ImageUsageFlagBits::eColorAttachment | vk::ImageUsageFlagBits::eTransferSrc,
                VMA_MEMORY_USAGE_GPU_ONLY);

            data.image_views[i] = device->createImageViewUnique(vk::ImageViewCreateInfo{
                {}, data.images[i].get(), vk::ImageViewType::e2D, OUTPUT_FORMAT, {},
                vk::ImageSubresourceRange{
                    vk::ImageAspectFlagBits::eColor,
                    /* base mipmap level */ 0, /* level count */ 1,
                    /* base array layer */ 0, /* layer count */ 1
                }
            });

            views[i] = data.image_views[i].get();
        }

        data.framebuffer = device->createFramebufferUnique(vk::FramebufferCreateInfo{
            {}, render_pass.get(),
            /* attachments */ static_cast<uint32_t>(views.size()), views.data(),
            resampling_resolution, resampling_resolution, /* layers */ 1
        });
    }
}

void Resampler::build_pipeline()
{
    auto create_shader_module = [this]
        (std::vector<std::byte> source)
    {
        vk::ShaderModuleCreateInfo info{{}, source.size(), reinterpret_cast<const uint32_t*>(source.data())};
        return device->createShaderModuleUnique(info);
    };

    auto vert_source = VkHelpers::read_shader("resampling.vert");
    auto vert_module = create_shader_module(vert_source);
    vk::PipelineShaderStageCreateInfo vert_info
        {{}, vk::ShaderStageFlagBits::eVertex, vert_module.get(), "main"};


    auto frag_source = VkHelpers::read_shader("resampling.frag");
    auto frag_module = create_shader_module(frag_source);
    vk::PipelineShaderStageCreateInfo frag_info
        {{}, vk::ShaderStageFlagBits::eFragment, frag_module.get(), "main"};

    std::array<vk::PipelineShaderStageCreateInfo, 2> shader_stages{vert_info, frag_info};

    vk::PipelineVertexInputStateCreateInfo vertex_input_info{
        {},
        1, &vertex_input_binding_description,
        uint32_t(vertex_input_attribute_descriptions.size()), vertex_input_attribute_descriptions.data()
    };

    vk::PipelineInputAssemblyStateCreateInfo triangle_input_assembly_info{
        {}, vk::PrimitiveTopology::eTriangleList, false
    };

    vk::Viewport viewport{0, 0, float(resampling_resolution), float(resampling_resolution), 0, 1};
    vk::Rect2D scissor{{0,                     0},
                       {resampling_resolution, resampling_resolution}};

    vk::PipelineViewportStateCreateInfo viewport_info{{}, 1, &viewport, 1, &scissor};

    vk::PipelineRasterizationStateCreateInfo rasterizer_info{
        {},
        /* depth clamp */ false,
        /* rasterizer discard */ false,
        vk::PolygonMode::eFill, vk::CullModeFlagBits::eNone, vk::FrontFace::eClockwise,
        /* depth bias */ false, 0, 0, 0,
        /* line width */ 1
    };

    vk::PipelineMultisampleStateCreateInfo multisample_info{
        {},
        vk::SampleCountFlagBits::e1,
        /* sample shading */ false, 1.f,
        /* sample mask */ nullptr,
        /* alpha to coverage */ false,
        /* alpha to one */ false
    };

    vk::PipelineColorBlendAttachmentState color_blend_attachment_state{
        false,
        vk::BlendFactor::eOne, vk::BlendFactor::eZero, vk::BlendOp::eAdd,
        vk::BlendFactor::eOne, vk::BlendFactor::eZero, vk::BlendOp::eAdd,
        vk::ColorComponentFlagBits::eR | vk::ColorComponentFlagBits::eG
            | vk::ColorComponentFlagBits::eB | vk::ColorComponentFlagBits::eA
    };

    std::array color_blend_attachment_states{color_blend_attachment_state, color_blend_attachment_state};

    vk::PipelineColorBlendStateCreateInfo color_blend_state_create_info{
        {}, false, vk::LogicOp::eCopy,
        static_cast<uint32_t>(color_blend_attachment_states.size()), color_blend_attachment_states.data(),
        {0, 0, 0, 0}
    };

    vk::DescriptorSetLayoutBinding ubo_layout_binding{
        /* binding */ 0,
        vk::DescriptorType::eUniformBuffer,
        /* descriptor count */ 1,
        vk::ShaderStageFlagBits::eVertex,
        nullptr
    };

    auto descriptor_set_layout = device->createDescriptorSetLayoutUnique(vk::DescriptorSetLayoutCreateInfo{
        {}, 1, &ubo_layout_binding
    });

    pipeline_layout = device->createPipelineLayoutUnique(vk::PipelineLayoutCreateInfo{
        {},
        /* set layouts */ 1, &descriptor_set_layout.get(),
        /* push constant ranges */ 0, nullptr
    });

    std::array attachment_descriptions{
        vk::AttachmentDescription{
            {}, OUTPUT_FORMAT, vk::SampleCountFlagBits::e1, vk::AttachmentLoadOp::eClear,
            vk::AttachmentStoreOp::eStore, vk::AttachmentLoadOp::eDontCare, vk::AttachmentStoreOp::eDontCare,
            vk::ImageLayout::eUndefined, vk::ImageLayout::eColorAttachmentOptimal
        },
        vk::AttachmentDescription{
            {}, OUTPUT_FORMAT, vk::SampleCountFlagBits::e1, vk::AttachmentLoadOp::eClear,
            vk::AttachmentStoreOp::eStore, vk::AttachmentLoadOp::eDontCare, vk::AttachmentStoreOp::eDontCare,
            vk::ImageLayout::eUndefined, vk::ImageLayout::eColorAttachmentOptimal
        }
    };

    std::array attachment_references{
        vk::AttachmentReference{0, vk::ImageLayout::eColorAttachmentOptimal},
        vk::AttachmentReference{1, vk::ImageLayout::eColorAttachmentOptimal}
    };

    vk::SubpassDescription subpass_description{
        {}, vk::PipelineBindPoint::eGraphics,
        0, nullptr,
        static_cast<uint32_t>(attachment_references.size()), attachment_references.data()
    };

    render_pass = device->createRenderPassUnique(vk::RenderPassCreateInfo{
        {},
        static_cast<uint32_t>(attachment_descriptions.size()), attachment_descriptions.data(),
        1, &subpass_description
    });

    triangle_pipeline = device->createGraphicsPipelineUnique({}, vk::GraphicsPipelineCreateInfo{
        {}, 2, shader_stages.data(),
        &vertex_input_info,
        &triangle_input_assembly_info,
        nullptr,
        &viewport_info,
        &rasterizer_info,
        &multisample_info,
        nullptr,
        &color_blend_state_create_info,
        nullptr,
        pipeline_layout.get(),
        render_pass.get(),
        0,
        {},
        -1
    });


    vk::PipelineInputAssemblyStateCreateInfo line_input_assembly_info{
        {}, vk::PrimitiveTopology::eLineStrip, false
    };

    line_pipeline = device->createGraphicsPipelineUnique({}, vk::GraphicsPipelineCreateInfo{
        {}, 2, shader_stages.data(),
        &vertex_input_info,
        &line_input_assembly_info,
        nullptr,
        &viewport_info,
        &rasterizer_info,
        &multisample_info,
        nullptr,
        &color_blend_state_create_info,
        nullptr,
        pipeline_layout.get(),
        render_pass.get(),
        0,
        {},
        -1
    });

    {
        VmaAllocatorCreateInfo info {
            .flags = {},
            .physicalDevice = physical_device,
            .device = device.get(),

            .preferredLargeHeapBlockSize = {},
            .pAllocationCallbacks = {},
            .pDeviceMemoryCallbacks = {},
            .frameInUseCount = 1,
            .pHeapSizeLimit = {},
            .pVulkanFunctions = {},
            .pRecordSettings = {},

            .instance = vk_instance.get(),
            .vulkanApiVersion = VK_API_VERSION_1_2,
        };
        vmaCreateAllocator(&info, &allocator);

        deferred_allocator_destroy =
            decltype(deferred_allocator_destroy)(&allocator,
                [](void* alloc)
                {
                    vmaDestroyAllocator(*reinterpret_cast<VmaAllocator*>(alloc));
                });
    }

    {
        uniform_buffer = UniqueVmaBuffer(allocator, sizeof(UniformBufferObject),
            vk::BufferUsageFlagBits::eUniformBuffer, VMA_MEMORY_USAGE_CPU_TO_GPU);

        UniformBufferObject ubo{
            float(resampling_resolution - 1) / float(resampling_resolution)
        };

        auto mapped = uniform_buffer.map();
        std::memcpy(mapped, &ubo, sizeof(ubo));
        uniform_buffer.unmap();
    }

    {
        std::array pool_sizes{
            vk::DescriptorPoolSize{vk::DescriptorType::eUniformBuffer, 1}
        };
        descriptor_pool = device->createDescriptorPoolUnique(vk::DescriptorPoolCreateInfo{
            {},
            /* max sets */ 1,
            /* pool sizes */ static_cast<uint32_t>(pool_sizes.size()), pool_sizes.data()
        });

        // Automatically destroyed with the pool
        descriptor_set = device->allocateDescriptorSets(vk::DescriptorSetAllocateInfo{
            descriptor_pool.get(),
            1,
            &descriptor_set_layout.get()
        }).front();

        vk::DescriptorBufferInfo ubo_info{
            uniform_buffer.get(),
            0,
            sizeof(UniformBufferObject)
        };

        device->updateDescriptorSets({
            vk::WriteDescriptorSet{
                descriptor_set,
                /* dst binding */ 0,
                /* dst array element */ 0,
                /* descriptor count */ 1,
                vk::DescriptorType::eUniformBuffer,
                nullptr,
                &ubo_info,
                nullptr
            }
        }, {});
    }
}

struct PatchVertexData
{
    std::vector<Vertex> vertices;
    std::vector<uint32_t> triangles;
    std::vector<uint32_t> line_strip;
    uint32_t bottom_right_index{};
};

PatchVertexData build_vertex_data(const QuadPatch& patch, const std::vector<MappingElement>& mapping)
{
    std::unordered_map<HashableCoords, uint32_t> indices;
    PatchVertexData result;

    {
        uint32_t idx = 0;
        result.vertices.reserve(mapping.size());

        for (auto[M, m] : mapping)
        {
            const auto&[m_x, m_y] = m;
            const auto&[x, y, z] = M;
            result.vertices.push_back({{float(m_x), float(m_y)}, {float(x), float(y), float(z)}});

            if (m_x == 1.f && m_y == 1.f)
            {
                result.bottom_right_index = idx;
            }

            indices.emplace(M, idx++);
        }

        result.line_strip.reserve(patch.boundary[1].size() + patch.boundary[2].size());

        for (std::size_t j = 1; j <= 2; ++j)
        {
            for (auto& v : patch.boundary[j])
            {
                result.line_strip.push_back(indices[v]);
            }
        }
    }

    result.triangles.reserve(patch.triangles.size());
    for (auto& tri : patch.triangles)
    {
        auto verts = triangle_verts(tri);
        std::array fields{&ThickTriangle::a, &ThickTriangle::b, &ThickTriangle::c};
        for (std::size_t i = 0; i < verts.size(); ++i)
        {
            auto index = indices[verts[i]];
            result.triangles.push_back(index);
            const ThickVertex& v = tri.*fields[i];
            result.vertices[index].normal << float(v.nx), float(v.ny), float(v.nz);
            result.vertices[index].uv << float(v.u), float(v.v);
        }
    }


    return result;
}

template<class T>
UniqueVmaBuffer make_buffer(VmaAllocator allocator, const std::vector<T>& data, vk::BufferUsageFlagBits usage)
{
    auto result = UniqueVmaBuffer(allocator, data.size() * sizeof(data[0]), usage, VMA_MEMORY_USAGE_CPU_TO_GPU);

    auto mapped = result.map();
    std::memcpy(mapped, data.data(), sizeof(data[0]) * data.size());
    result.unmap();

    return result;
}

std::vector<float> Resampler::resample(const QuadPatch& patch,
    const std::vector<MappingElement>& mapping, std::size_t thread_idx)
{
    auto[vertices, triangles, line_strip, bottom_right_index] = build_vertex_data(patch, mapping);

    auto& data = per_thread_datum[thread_idx];


    auto vertex_buffer = make_buffer(allocator, vertices, vk::BufferUsageFlagBits::eVertexBuffer);
    auto triangle_buffer = make_buffer(allocator, triangles, vk::BufferUsageFlagBits::eIndexBuffer);
    auto line_buffer = make_buffer(allocator, line_strip, vk::BufferUsageFlagBits::eIndexBuffer);


    build_command_buffer(vertex_buffer.get(), triangle_buffer.get(), line_buffer.get(),
        static_cast<uint32_t>(triangles.size()), static_cast<uint32_t>(line_strip.size()), thread_idx);

    vk::SubmitInfo submit_info{
        /* wait semaphores */ 0, nullptr, nullptr,
        1, &data.command_buffer.get(),
        /* signal semaphores */ 0, nullptr
    };
    
    if (data.queue.submit(1, &submit_info, nullptr) != vk::Result::eSuccess)
    {
        throw std::runtime_error("Resampling didnt succeed!");
    }

    data.queue.waitIdle();

    std::vector<float> result;

    {
        result.resize(resampling_resolution*resampling_resolution*8);

        auto current0 = reinterpret_cast<const float*>(data.image_staging[0].map());
        auto current1 = reinterpret_cast<const float*>(data.image_staging[1].map());
        for (std::size_t i = 0; i < resampling_resolution*resampling_resolution; ++i)
        {
            std::memcpy(result.data() + 8*i,     current0, 4*sizeof(float));
            std::memcpy(result.data() + 8*i + 4, current1, 4*sizeof(float));
            current0 += 4;
            current1 += 4;
        }
        data.image_staging[0].unmap();
        data.image_staging[1].unmap();

        // THIS IS A KOSTYL
        // TODO: Think about this
        // bottom right corner does not get rendered due to rasterization rules
        // this can be fixed with a teeeeny-tiny upscaling, but this might result in
        // seams due to interpolation :(

        result[result.size() - 8] = vertices[bottom_right_index].position.x();
        result[result.size() - 7] = vertices[bottom_right_index].position.y();
        result[result.size() - 6] = vertices[bottom_right_index].position.z();

        result[result.size() - 5] = vertices[bottom_right_index].uv.x();
        result[result.size() - 4] = vertices[bottom_right_index].uv.y();

        result[result.size() - 3] = vertices[bottom_right_index].normal.x();
        result[result.size() - 2] = vertices[bottom_right_index].normal.y();
        result[result.size() - 1] = vertices[bottom_right_index].normal.z();

    }

    return result;
}

void Resampler::build_command_buffer(vk::Buffer vertex_buffer, vk::Buffer triangle_buffer, vk::Buffer line_buffer,
    uint32_t triangle_indices_count, uint32_t line_indices_count, std::size_t thread_idx)
{
    auto& data = per_thread_datum[thread_idx];
    auto& cb = data.command_buffer;

    cb = std::move(device->allocateCommandBuffersUnique(vk::CommandBufferAllocateInfo{
        command_pool.get(), vk::CommandBufferLevel::ePrimary, 1}).front());

    cb->begin(vk::CommandBufferBeginInfo{});

    for (auto& image : data.images)
    {
        vk::ImageMemoryBarrier barrier{
            {}, vk::AccessFlagBits::eColorAttachmentWrite,
            vk::ImageLayout::eUndefined, vk::ImageLayout::eColorAttachmentOptimal,
            VK_QUEUE_FAMILY_IGNORED,
            VK_QUEUE_FAMILY_IGNORED,
            image.get(),
            vk::ImageSubresourceRange{
                vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1
            }
        };
        cb->pipelineBarrier(
            vk::PipelineStageFlagBits::eTopOfPipe, vk::PipelineStageFlagBits::eColorAttachmentOutput,
            {}, 0, nullptr, 0, nullptr, 1, &barrier);
    }

    std::array clear_values{
        vk::ClearValue{std::array{0.f, 0.f, 0.f, 0.f}},
        vk::ClearValue{std::array{0.f, 0.f, 0.f, 0.f}},
        vk::ClearValue{std::array{0.f, 0.f, 0.f, 0.f}}
    };


    cb->beginRenderPass(vk::RenderPassBeginInfo{
        render_pass.get(), data.framebuffer.get(),
        vk::Rect2D{{0, 0}, {resampling_resolution, resampling_resolution}},
        static_cast<uint32_t>(clear_values.size()), clear_values.data()
    }, vk::SubpassContents::eInline);

    {
        cb->bindPipeline(vk::PipelineBindPoint::eGraphics, triangle_pipeline.get());

        cb->bindVertexBuffers(0, {vertex_buffer}, {0});
        cb->bindDescriptorSets(vk::PipelineBindPoint::eGraphics,
            pipeline_layout.get(), 0, {descriptor_set}, {});
        cb->bindIndexBuffer(triangle_buffer, 0, vk::IndexType::eUint32);

        cb->drawIndexed(triangle_indices_count, 1, 0, 0, 0);
    }

    {
        cb->bindPipeline(vk::PipelineBindPoint::eGraphics, line_pipeline.get());

        cb->bindVertexBuffers(0, {vertex_buffer}, {0});
        cb->bindDescriptorSets(vk::PipelineBindPoint::eGraphics,
            pipeline_layout.get(), 0, {descriptor_set}, {});
        cb->bindIndexBuffer(line_buffer, 0, vk::IndexType::eUint32);

        cb->drawIndexed(line_indices_count, 1, 0, 0, 0);
    }

    cb->endRenderPass();


    for (auto& image : data.images)
    {
        vk::ImageMemoryBarrier barrier{
            vk::AccessFlagBits::eColorAttachmentWrite, vk::AccessFlagBits::eTransferRead,
            vk::ImageLayout::eColorAttachmentOptimal, vk::ImageLayout::eTransferSrcOptimal,
            VK_QUEUE_FAMILY_IGNORED,
            VK_QUEUE_FAMILY_IGNORED,
            image.get(),
            vk::ImageSubresourceRange{
                vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1
            }
        };
        cb->pipelineBarrier(
            vk::PipelineStageFlagBits::eColorAttachmentOutput, vk::PipelineStageFlagBits::eTransfer,
            {}, 0, nullptr, 0, nullptr, 1, &barrier);
    }

    for (std::size_t i = 0; i < data.images.size(); ++i)
    {
        vk::BufferImageCopy region{
            0, resampling_resolution, resampling_resolution,
            vk::ImageSubresourceLayers{
                vk::ImageAspectFlagBits::eColor, 0, 0, 1
            },
            vk::Offset3D{0, 0, 0}, vk::Extent3D{resampling_resolution, resampling_resolution, 1}
        };
        cb->copyImageToBuffer(data.images[i].get(), vk::ImageLayout::eTransferSrcOptimal, data.image_staging[i].get(),
            1, &region);
    }

    cb->end();
}
