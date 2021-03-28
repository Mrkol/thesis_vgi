#include "Resampler.hpp"

#include <fstream>

#include "../DataTypes.hpp"


std::vector<char> read_shader(std::string_view name)
{
    auto path = std::filesystem::current_path();
    path /= "shaders";
    path /= name.data() + std::string(".spv");
    std::ifstream file{path, std::ios::ate | std::ios::binary};

    if (!file.is_open())
    {
        throw std::runtime_error("Shader is missing!");
    }

    std::vector<char> result(file.tellg());
    file.seekg(0);
    file.read(result.data(), result.size());

    return result;
}

struct Vertex
{
    MappingCoords mapping_position;
    HashableCoords position;
};

constexpr vk::VertexInputBindingDescription vertex_input_binding_description
    {0, sizeof(Vertex), vk::VertexInputRate::eVertex};

constexpr std::array vertex_input_attribute_descriptions{
    vk::VertexInputAttributeDescription
        {/* binding */ 0, /* location */ 0, vk::Format::eR64G64Sfloat, offsetof(Vertex, mapping_position)},
    vk::VertexInputAttributeDescription
        {/* binding */ 0, /* location */ 2, vk::Format::eR64G64B64Sfloat, offsetof(Vertex, position)}
};


Resampler::Resampler(const ResamplerConfig& config)
{
    if constexpr (!VALIDATION_LAYERS.empty())
    {
        auto layers = vk::enumerateInstanceLayerProperties();
        for (auto valid_layer : VALIDATION_LAYERS)
        {
            bool found = false;
            for (auto layer : layers)
            {
                if (std::string_view(layer.layerName) == valid_layer)
                {
                    found = true;
                    break;
                }
            }
            if (!found)
            {
                throw std::runtime_error("Validation layer " + std::string(valid_layer) + " not supported!");
            }
        }
    }

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


    command_pool = device->createCommandPoolUnique(vk::CommandPoolCreateInfo{
        {}, uint32_t(queue_idx)
    });

    BuildPipeline(config);

    auto command_buffers = device->allocateCommandBuffersUnique(vk::CommandBufferAllocateInfo{
        command_pool.get(), vk::CommandBufferLevel::ePrimary, uint32_t(config.thread_count)});

    for (std::size_t idx = 0; idx < config.thread_count; ++idx)
    {
        per_thread_datum[idx].command_buffer = std::move(command_buffers[idx]);
    }

    vk::ClearValue clear_value{std::array{0.f, 0.f, 0.f, 0.f}};

    for (auto& data : per_thread_datum)
    {
        auto& cb = data.command_buffer;

        cb->begin(vk::CommandBufferBeginInfo{});

        cb->beginRenderPass(vk::RenderPassBeginInfo{
            render_pass.get(), data.framebuffer.get(),
            vk::Rect2D{{0, 0}, {uint32_t(config.frequency), uint32_t(config.frequency)}},
            1, &clear_value
        }, vk::SubpassContents::eInline);

        cb->bindPipeline(vk::PipelineBindPoint::eGraphics, graphics_pipeline.get());

        cb->draw(3, 1, 0, 0);

        cb->endRenderPass();

        cb->end();
    }
}

void Resampler::BuildPipeline(const ResamplerConfig& config)
{
    auto create_shader_module = [this]
        (const std::vector<char>& source)
    {
        vk::ShaderModuleCreateInfo info{{}, source.size(), reinterpret_cast<const uint32_t*>(source.data())};
        return device->createShaderModuleUnique(info);
    };

    auto vert_source = read_shader("resampling.vert");
    auto vert_module = create_shader_module(vert_source);
    vk::PipelineShaderStageCreateInfo vert_info
        {{}, vk::ShaderStageFlagBits::eVertex, vert_module.get(), "resampling"};

    device->destroy(vert_module.get());

    auto frag_source = read_shader("resampling.frag");
    auto frag_module = create_shader_module(frag_source);
    vk::PipelineShaderStageCreateInfo frag_info
        {{}, vk::ShaderStageFlagBits::eFragment, frag_module.get(), "resampling"};

    std::array<vk::PipelineShaderStageCreateInfo, 2> shader_stages{vert_info, frag_info};

    vk::PipelineVertexInputStateCreateInfo vertex_input_info{
        {},
        1, &vertex_input_binding_description,
        uint32_t(vertex_input_attribute_descriptions.size()), vertex_input_attribute_descriptions.data()
    };

    vk::PipelineInputAssemblyStateCreateInfo input_assembly_info{{}, vk::PrimitiveTopology::eTriangleList, false};

    vk::Viewport viewport{0, 0, float(config.frequency), float(config.frequency), 0, 1};
    vk::Rect2D scissor{{0, 0}, {uint32_t(config.frequency), uint32_t(config.frequency)}};

    vk::PipelineViewportStateCreateInfo viewport_info{{}, 1, &viewport, 1, &scissor};

    vk::PipelineRasterizationStateCreateInfo rasterizer_info
        {{}, false, false, vk::PolygonMode::eFill, vk::CullModeFlagBits::eNone, vk::FrontFace::eClockwise,
         false, 0, 0, 0, 1};

    vk::PipelineColorBlendAttachmentState color_blend_attachment_state
        {false,
         vk::BlendFactor::eOne, vk::BlendFactor::eZero, vk::BlendOp::eAdd,
         vk::BlendFactor::eOne, vk::BlendFactor::eZero, vk::BlendOp::eAdd,
         vk::ColorComponentFlagBits::eR | vk::ColorComponentFlagBits::eG
            | vk::ColorComponentFlagBits::eB | vk::ColorComponentFlagBits::eA};

    vk::PipelineColorBlendStateCreateInfo color_blend_state_create_info
        {{}, false, vk::LogicOp::eCopy, 1, &color_blend_attachment_state, {0, 0, 0, 0}};

    vk::PipelineLayoutCreateInfo pipeline_layout_create_info
        {{}, 0, nullptr, 0, nullptr};

    auto pipeline_layout = device->createPipelineLayoutUnique(pipeline_layout_create_info);

    vk::AttachmentDescription attachment_description
        {{}, vk::Format::eA8B8G8R8SrgbPack32, vk::SampleCountFlagBits::e1, vk::AttachmentLoadOp::eClear,
         vk::AttachmentStoreOp::eStore, vk::AttachmentLoadOp::eDontCare, vk::AttachmentStoreOp::eDontCare,
         vk::ImageLayout::eUndefined, vk::ImageLayout::ePresentSrcKHR};

    vk::AttachmentReference attachment_reference{0, vk::ImageLayout::eColorAttachmentOptimal};
    vk::SubpassDescription subpass_description
        {{}, vk::PipelineBindPoint::eGraphics, 0, nullptr, 1, &attachment_reference};

    vk::RenderPassCreateInfo render_pass_create_info
        {{}, 1, &attachment_description, 1, &subpass_description};

    render_pass = device->createRenderPassUnique(render_pass_create_info);

    vk::GraphicsPipelineCreateInfo graphics_pipeline_create_info
        {{}, 2, shader_stages.data(),
         &vertex_input_info,
         &input_assembly_info,
         nullptr,
         &viewport_info,
         &rasterizer_info,
         nullptr,
         nullptr,
         &color_blend_state_create_info,
         nullptr,
         pipeline_layout.get(),
         render_pass.get(),
         0,
         {},
         -1};

    for (auto& data : per_thread_datum)
    {
        data.image = device->createImageUnique(vk::ImageCreateInfo{
            {}, vk::ImageType::e2D, OUTPUT_FORMAT,
            /* extents */ {uint32_t(config.frequency), uint32_t(config.frequency), 1},
            /* mip levels */ 1, /* array layers */ 1, vk::SampleCountFlagBits::e1,
            vk::ImageTiling::eOptimal, vk::ImageUsageFlagBits::eColorAttachment
        });

        auto requirements = device->getImageMemoryRequirements(data.image.get());
        auto memory = device->allocateMemoryUnique(vk::MemoryAllocateInfo{
            requirements.size,
            0 // TODO: figure this out
        });

        device->bindImageMemory(data.image.get(), memory.get(), 0);

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
            uint32_t(config.frequency), uint32_t(config.frequency), /* layers */ 1
        });
    }
}

std::vector<Vertex> read_vertex_data(const std::filesystem::path& quad, const std::filesystem::path& info)
{
    auto patch = read_plainfile(quad);
    std::unordered_map<HashableCoords, MappingCoords> mapping;

    {
        std::vector<std::pair<HashableCoords, MappingCoords>> raw_mapping(file_size(info));
        std::ifstream in{info, std::ios_base::binary};
        in.read(reinterpret_cast<char*>(raw_mapping.data()), raw_mapping.size());
        for (auto[M, m] : raw_mapping)
        {
            mapping.emplace(M, m);
        }
    }

    std::vector<Vertex> result;
    result.reserve(patch.size() * 3);
    for (auto& tri : patch)
    {
        auto verts = triangle_verts(tri);
        for (auto v : verts)
        {
            result.push_back({mapping[v], v});
        }
    }

    return result;
}

void Resampler::Resample(const std::filesystem::path& quad, const std::filesystem::path& info,
    const std::filesystem::path& output_dir, std::size_t thread_idx)
{
    auto vertices = read_vertex_data(quad, info);

    auto& data = per_thread_datum[thread_idx];

    auto vertex_buffer = device->createBufferUnique(vk::BufferCreateInfo{

    });

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
}
