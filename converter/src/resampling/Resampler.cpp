#include "Resampler.hpp"

#include <fstream>

#include "../DataTypes.hpp"
#include "VkHelpers.hpp"


struct Vertex
{
    struct Mapped
    {
        float m_x, m_y;
    } mapped;
    struct Position
    {
        float x, y, z;
    } position;
};

struct UniformBufferObject
{
    float frequency;
};

constexpr vk::VertexInputBindingDescription vertex_input_binding_description
    {0, sizeof(Vertex), vk::VertexInputRate::eVertex};

constexpr std::array vertex_input_attribute_descriptions{
    vk::VertexInputAttributeDescription
        {/* location */ 0, /* binding */ 0, vk::Format::eR32G32Sfloat, offsetof(Vertex, mapped)},
    vk::VertexInputAttributeDescription
        {/* location */ 1, /* binding */ 0, vk::Format::eR32G32B32Sfloat, offsetof(Vertex, position)}
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
        per_thread_datum[idx].rendered_fence = device->createFenceUnique(vk::FenceCreateInfo{});
    }


    command_pool = device->createCommandPoolUnique(vk::CommandPoolCreateInfo{{}, uint32_t(queue_idx)});

    build_pipeline();

    for (auto& data : per_thread_datum)
    {
        // TODO: Most GPUs dont support ImageTiling::eLinear with ImageUsageFlagBits::eColorAttachment!!!
        // gotta split it into 2 buffers and use one as a render target and the other one as GPU-CPU transfer buffer
        data.image = device->createImageUnique(vk::ImageCreateInfo{
            {}, vk::ImageType::e2D, OUTPUT_FORMAT,
            /* extents */ {resampling_resolution, resampling_resolution, 1},
            /* mip levels */ 1, /* array layers */ 1, vk::SampleCountFlagBits::e1,
            vk::ImageTiling::eLinear, vk::ImageUsageFlagBits::eColorAttachment
        });

        auto mem_reqs = device->getImageMemoryRequirements(data.image.get());
        data.image_memory = device->allocateMemoryUnique(vk::MemoryAllocateInfo{
            mem_reqs.size,
            VkHelpers::find_memory_type(physical_device, mem_reqs.memoryTypeBits,
                vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent)
        });

        device->bindImageMemory(data.image.get(), data.image_memory.get(), 0);

        data.image_view = device->createImageViewUnique(vk::ImageViewCreateInfo{
            {}, data.image.get(), vk::ImageViewType::e2D, OUTPUT_FORMAT, {},
            vk::ImageSubresourceRange{
                vk::ImageAspectFlagBits::eColor,
                /* base mipmap level */ 0, /* level count */ 1,
                /* base array layer */ 0, /* layer count */ 1
            }
        });

        data.framebuffer = device->createFramebufferUnique(vk::FramebufferCreateInfo{
            {}, render_pass.get(), /* attachment count */ 1, &data.image_view.get(),
            resampling_resolution, resampling_resolution, /* layers */ 1
        });
    }
}

void Resampler::build_pipeline()
{
    auto create_shader_module = [this]
        (const std::vector<char>& source)
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

    vk::PipelineColorBlendAttachmentState color_blend_attachment_state
        {false,
         vk::BlendFactor::eOne, vk::BlendFactor::eZero, vk::BlendOp::eAdd,
         vk::BlendFactor::eOne, vk::BlendFactor::eZero, vk::BlendOp::eAdd,
         vk::ColorComponentFlagBits::eR | vk::ColorComponentFlagBits::eG
             | vk::ColorComponentFlagBits::eB | vk::ColorComponentFlagBits::eA};

    vk::PipelineColorBlendStateCreateInfo color_blend_state_create_info
        {{}, false, vk::LogicOp::eCopy, 1, &color_blend_attachment_state, {0, 0, 0, 0}};

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

    vk::AttachmentDescription attachment_description
        {{}, OUTPUT_FORMAT, vk::SampleCountFlagBits::e1, vk::AttachmentLoadOp::eClear,
         vk::AttachmentStoreOp::eStore, vk::AttachmentLoadOp::eDontCare, vk::AttachmentStoreOp::eDontCare,
         vk::ImageLayout::eUndefined, vk::ImageLayout::ePresentSrcKHR};

    vk::AttachmentReference attachment_reference
        {0, vk::ImageLayout::eColorAttachmentOptimal};
    vk::SubpassDescription subpass_description
        {{}, vk::PipelineBindPoint::eGraphics, 0, nullptr, 1, &attachment_reference};

    render_pass = device->createRenderPassUnique(vk::RenderPassCreateInfo{
        {},
        1, &attachment_description,
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
        uniform_buffer = device->createBufferUnique(vk::BufferCreateInfo{
            {},
            sizeof(UniformBufferObject),
            vk::BufferUsageFlagBits::eUniformBuffer,
            vk::SharingMode::eExclusive
        });

        auto memory_requirements = device->getBufferMemoryRequirements(uniform_buffer.get());
        uniform_buffer_memory = device->allocateMemoryUnique(vk::MemoryAllocateInfo{
            memory_requirements.size,
            VkHelpers::find_memory_type(physical_device, memory_requirements.memoryTypeBits,
                vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent)
        });

        device->bindBufferMemory(uniform_buffer.get(), uniform_buffer_memory.get(), 0);

        UniformBufferObject ubo{
            float(resampling_resolution - 1) / float(resampling_resolution)
        };

        void* mapped_data;
        if (device->mapMemory(uniform_buffer_memory.get(), 0, memory_requirements.size, {}, &mapped_data) != vk::Result::eSuccess)
        {
            throw std::runtime_error("Unable to map buffer memory!");
        }
        std::memcpy(mapped_data, &ubo, sizeof(ubo));
        device->unmapMemory(uniform_buffer_memory.get());
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
        for (auto v : verts)
        {
            result.triangles.push_back(indices[v]);
        }
    }


    return result;
}

struct BufferAndMemory
{
    vk::UniqueDeviceMemory memory;
    vk::UniqueBuffer buffer;
};

template<class T>
BufferAndMemory make_buffer(const vk::PhysicalDevice& physical_device, vk::Device* device,
    const std::vector<T>& data, vk::BufferUsageFlagBits usage)
{
    auto buffer = device->createBufferUnique(vk::BufferCreateInfo{
        {}, sizeof(data[0]) * data.size(),
        usage,
        vk::SharingMode::eExclusive
    });

    auto memory_requirements = device->getBufferMemoryRequirements(buffer.get());
    auto memory = device->allocateMemoryUnique(vk::MemoryAllocateInfo{
        memory_requirements.size,
        VkHelpers::find_memory_type(physical_device, memory_requirements.memoryTypeBits,
            vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent)
    });

    device->bindBufferMemory(buffer.get(), memory.get(), 0);

    void* mapped_data;
    if (device->mapMemory(memory.get(), 0, memory_requirements.size, {}, &mapped_data) != vk::Result::eSuccess)
    {
        throw std::runtime_error("Unable to map buffer memory!");
    }
    std::memcpy(mapped_data, data.data(), sizeof(data[0]) * data.size());
    device->unmapMemory(memory.get());

    return {std::move(memory), std::move(buffer)};
}

std::vector<std::array<float, 3>> Resampler::resample(const QuadPatch& patch,
    const std::vector<MappingElement>& mapping, std::size_t thread_idx)
{
    auto[vertices, triangles, line_strip] = build_vertex_data(patch, mapping);

    auto& data = per_thread_datum[thread_idx];

    device->resetFences({data.rendered_fence.get()});

    auto[vertex_memory, vertex_buffer] =
        make_buffer(physical_device, &device.get(),
            vertices, vk::BufferUsageFlagBits::eVertexBuffer);

    auto[triangle_memory, triangle_buffer] =
        make_buffer(physical_device, &device.get(),
            triangles, vk::BufferUsageFlagBits::eIndexBuffer);

    auto[line_memory, line_buffer] =
        make_buffer(physical_device, &device.get(),
            line_strip, vk::BufferUsageFlagBits::eIndexBuffer);


    build_command_buffer(vertex_buffer.get(), triangle_buffer.get(), line_buffer.get(),
        static_cast<uint32_t>(triangles.size()), static_cast<uint32_t>(line_strip.size()), thread_idx);

    vk::SubmitInfo submit_info{
        /* wait semaphores */ 0, nullptr, nullptr,
        1, &data.command_buffer.get(),
        /* signal semaphores */ 0, nullptr
    };
    
    if (data.queue.submit(1, &submit_info, data.rendered_fence.get()) != vk::Result::eSuccess
        || device->waitForFences({data.rendered_fence.get()}, true, 1'000'000'000) != vk::Result::eSuccess)
    {
        throw std::runtime_error("Resampling didnt succeed!");
    }

    auto layout = device->getImageSubresourceLayout(data.image.get(),
        vk::ImageSubresource{vk::ImageAspectFlagBits::eColor});

    const char* current_byte;
    if (device->mapMemory(data.image_memory.get(), 0, layout.size, {},
        reinterpret_cast<void**>(const_cast<char**>(&current_byte))) != vk::Result::eSuccess)
    {
        throw std::runtime_error("Unable to map output buffer result!");
    }

    current_byte += layout.offset;

    std::vector<std::array<float, 3>> result;
    result.reserve(resampling_resolution*resampling_resolution);

    for (std::size_t y = 0; y < resampling_resolution; ++y)
    {
        const char* current =  current_byte;
        for (std::size_t x = 0; x < resampling_resolution; ++x)
        {
            result.push_back(*reinterpret_cast<const std::array<float, 3>*>(current));
            // skip alpha
            current += 4*sizeof(float);
        }
        current_byte += layout.rowPitch;
    }

    // THIS IS A KOSTYL
    // TODO: Think about this
    // bottom right corner does not get rendered due to rasterization rules
    // this can be fixed with a teeeeny-tiny upscaling, but this might result in
    // seams due to interpolation :(

    result.back()[0] = static_cast<float>(get<0>(patch.boundary[1].back()));
    result.back()[1] = static_cast<float>(get<1>(patch.boundary[1].back()));
    result.back()[2] = static_cast<float>(get<2>(patch.boundary[1].back()));

    device->unmapMemory(data.image_memory.get());

    return result;
}

void Resampler::build_command_buffer(vk::Buffer vertex_buffer, vk::Buffer triangle_buffer, vk::Buffer line_buffer,
    uint32_t triangle_indices_count, uint32_t line_indices_count, std::size_t thread_idx)
{
    vk::ClearValue clear_value{std::array{0.f, 0.f, 0.f, 0.f}};

    auto& data = per_thread_datum[thread_idx];
    auto& cb = data.command_buffer;

    cb = std::move(device->allocateCommandBuffersUnique(vk::CommandBufferAllocateInfo{
        command_pool.get(), vk::CommandBufferLevel::ePrimary, 1}).front());

    cb->begin(vk::CommandBufferBeginInfo{});

    cb->beginRenderPass(vk::RenderPassBeginInfo{
        render_pass.get(), data.framebuffer.get(),
        vk::Rect2D{{0, 0}, {resampling_resolution, resampling_resolution}},
        1, &clear_value
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

    cb->end();
}
