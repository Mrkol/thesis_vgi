#include "Renderer.hpp"

#define VMA_IMPLEMENTATION
#include <vk_mem_alloc.h>


#include <iostream>
#include <unordered_set>

#include "SceneObjectBase.hpp"


constexpr std::array DEVICE_EXTENSIONS{VK_KHR_SWAPCHAIN_EXTENSION_NAME};


QueueFamilyIndices chose_queue_families(const vk::PhysicalDevice& device, const vk::SurfaceKHR& surface)
{
    auto queues = device.getQueueFamilyProperties();

    constexpr uint32_t INVALID = std::numeric_limits<uint32_t>::max();
    QueueFamilyIndices result{INVALID, INVALID};

    uint32_t queue_idx = 0;
    for (const auto& queue : queues)
    {
        if (queue.queueFlags & vk::QueueFlagBits::eGraphics)
        {
            result.graphics_queue_idx = queue_idx;
        }

        if (device.getSurfaceSupportKHR(queue_idx , surface))
        {
            result.presentation_queue_idx = queue_idx;
        }

        ++queue_idx;
    }

    if (result.graphics_queue_idx == INVALID
        || result.presentation_queue_idx == INVALID)
    {
        throw std::runtime_error("Chosen physical device does not support a graphics queue!");
    }

    return result;
}

vk::SurfaceFormatKHR chose_surface_format(const vk::PhysicalDevice& device, const vk::SurfaceKHR& surface)
{
    auto formats = device.getSurfaceFormatsKHR(surface);

    if (formats.empty())
    {
        throw std::runtime_error("Device does not support any surface formats!");
    }

    auto found = std::find_if(formats.begin(), formats.end(),
        [](const vk::SurfaceFormatKHR& format)
        {
            return format.format == vk::Format::eB8G8R8A8Srgb
                && format.colorSpace == vk::ColorSpaceKHR::eSrgbNonlinear;
        });

    return found != formats.end() ? *found : formats[0];
}

vk::PresentModeKHR chose_present_mode(const vk::PhysicalDevice& device, const vk::SurfaceKHR& surface)
{
    auto modes = device.getSurfacePresentModesKHR(surface);

    if (modes.empty())
    {
        throw std::runtime_error("Device doesn't support any present modes!");
    }

    return std::find(modes.begin(), modes.end(), vk::PresentModeKHR::eMailbox) != modes.end() ?
        vk::PresentModeKHR::eMailbox : vk::PresentModeKHR::eFifo;
}

Renderer::Renderer(vk::Instance& instance, vk::UniqueSurfaceKHR surface,
    std::span<const char* const> validation_layers,
    fu2::unique_function<vk::Extent2D()> res_provider)
    : resolution_provider{std::move(res_provider)}
    , vulkan_instance{&instance}
    , display_surface{std::move(surface)}
{
    // TODO: Proper device selection
    physical_device = vulkan_instance->enumeratePhysicalDevices().front();

    auto props = physical_device.getProperties();
    std::cout << "Max bound desc sets: " << props.limits.maxBoundDescriptorSets << std::endl;

    queue_family_indices = chose_queue_families(physical_device, display_surface.get());    

    // kostyl due to bad API design
    float queue_priority = 1.0f;
    std::vector<vk::DeviceQueueCreateInfo> queue_create_infos;
    {
        std::unordered_set<uint32_t> unique_families{
            queue_family_indices.graphics_queue_idx,
            queue_family_indices.presentation_queue_idx
        };

        queue_create_infos.reserve(unique_families.size());
        for (auto idx : unique_families)
        {
            queue_create_infos.emplace_back(vk::DeviceQueueCreateInfo{{}, idx, 1, &queue_priority});
        }
    }


    vk::PhysicalDeviceFeatures features {
    };

    // NOTE: newer vulkan implementations ignore the layers here
    device = physical_device.createDeviceUnique(vk::DeviceCreateInfo{
        {},
        static_cast<uint32_t>(queue_create_infos.size()), queue_create_infos.data(),
        static_cast<uint32_t>(validation_layers.size()), validation_layers.data(),
        static_cast<uint32_t>(DEVICE_EXTENSIONS.size()), DEVICE_EXTENSIONS.data(),
        &features
    });
    
    graphics_queue = device->getQueue(queue_family_indices.graphics_queue_idx, 0);
    present_queue = device->getQueue(queue_family_indices.presentation_queue_idx, 0);

    {
        VmaAllocatorCreateInfo info{
            .flags = {},
            .physicalDevice = physical_device,
            .device = device.get(),

            .preferredLargeHeapBlockSize = {},
            .pAllocationCallbacks = {},
            .pDeviceMemoryCallbacks = {},
            .frameInUseCount = MAX_FRAMES_IN_FLIGHT,
            .pHeapSizeLimit = {},
            .pVulkanFunctions = {},
            .pRecordSettings = {},

            .instance = instance,
            .vulkanApiVersion = VK_API_VERSION_1_2, // TODO: global constant for this
        };
        vmaCreateAllocator(&info, &allocator);

        deferred_allocator_destroy =
            decltype(deferred_allocator_destroy)(&allocator,
                [](void* alloc)
                {
                    vmaDestroyAllocator(*reinterpret_cast<VmaAllocator*>(alloc));
                });
    }

    command_pool = device->createCommandPoolUnique(vk::CommandPoolCreateInfo{
        {}, queue_family_indices.graphics_queue_idx
    });

    create_swapchain();

    // TODO: This really confuses me...
    std::size_t max_objects = 100;
    // 2 sets: one global, one per-model
    std::size_t max_sets = MAX_FRAMES_IN_FLIGHT * 2;
    std::array pool_sizes{
        vk::DescriptorPoolSize{vk::DescriptorType::eUniformBuffer, static_cast<uint32_t>(max_objects * max_sets)}
    };
    global_descriptor_pool = device->createDescriptorPoolUnique(vk::DescriptorPoolCreateInfo{
        {},
        /* max sets */ static_cast<uint32_t>(max_sets),
        /* pool sizes */ static_cast<uint32_t>(pool_sizes.size()), pool_sizes.data()
    });


    scene = std::make_unique<Scene>(this, PipelineCreationInfo{
        swapchain_data.render_pass.get(),
        swapchain_data.extent
    });

    for (auto& data : per_inflight_frame_data)
    {
        data.image_available_semaphore = device->createSemaphoreUnique(vk::SemaphoreCreateInfo{});
        data.rendering_finished_semaphore = device->createSemaphoreUnique(vk::SemaphoreCreateInfo{});
        data.in_flight_fence = device->createFenceUnique(vk::FenceCreateInfo{vk::FenceCreateFlagBits::eSignaled});
    }



    // TODO: remove
    for (std::size_t i = 0; i < swapchain_data.elements.size(); ++i)
    {
        record_commands(i);
    }
}

void Renderer::render()
{
    auto& data = per_inflight_frame_data[current_frame_idx];

    device->waitForFences({data.in_flight_fence.get()}, true, std::numeric_limits<uint64_t>::max());

    scene->tick();

    uint32_t idx;
    {
        auto result = device->acquireNextImageKHR(swapchain_data.swapchain.get(),
            std::numeric_limits<uint64_t>::max(), data.image_available_semaphore.get(), {}, &idx);

        if (result == vk::Result::eErrorOutOfDateKHR)
        {
            create_swapchain();
            return;
        }
        else if (result != vk::Result::eSuccess && result != vk::Result::eSuboptimalKHR)
        {
            throw std::runtime_error("Failed to acquire swapchain image!");
        }
    }

    auto& swapchain_element = swapchain_data.elements[idx];

    if (swapchain_element.image_fence)
    {
        device->waitForFences({swapchain_element.image_fence}, true, std::numeric_limits<uint64_t>::max());
    }

    swapchain_element.image_fence = data.in_flight_fence.get();

    vk::PipelineStageFlags stage_flags = vk::PipelineStageFlagBits::eColorAttachmentOutput;

    std::array wait_semos{data.image_available_semaphore.get()};
    std::array signal_semos{data.rendering_finished_semaphore.get()};

    std::array submits{
        vk::SubmitInfo{
            static_cast<uint32_t>(wait_semos.size()), wait_semos.data(),
            &stage_flags,
            1, &swapchain_element.command_buffer.get(),
            static_cast<uint32_t>(signal_semos.size()), signal_semos.data()
        }
    };

    device->resetFences({data.in_flight_fence.get()});
    graphics_queue.submit(
        static_cast<uint32_t>(submits.size()), submits.data(), data.in_flight_fence.get());

    {
        std::array swapchains{swapchain_data.swapchain.get()};

        vk::PresentInfoKHR present_info{
            static_cast<uint32_t>(signal_semos.size()), signal_semos.data(),
            static_cast<uint32_t>(swapchains.size()), swapchains.data(),
            &idx
        };

        auto result = present_queue.presentKHR(&present_info);

        if (result == vk::Result::eErrorOutOfDateKHR || result == vk::Result::eSuboptimalKHR
            || swapchain_data.needs_recreation)
        {
            create_swapchain();
        }
        else if (result != vk::Result::eSuccess)
        {
            throw std::runtime_error("Graphics queue submission failed!");
        }

    }

    ++current_frame_idx;
    current_frame_idx %= MAX_FRAMES_IN_FLIGHT;
}

vk::Extent2D Renderer::chose_swap_extent(const vk::SurfaceCapabilitiesKHR& capabilities)
{
    if (capabilities.currentExtent.width != std::numeric_limits<uint32_t>::max()) {
        return capabilities.currentExtent;
    }

    VkExtent2D actualExtent = resolution_provider();

    actualExtent.width = std::max(capabilities.minImageExtent.width, std::min(capabilities.maxImageExtent.width, actualExtent.width));
    actualExtent.height = std::max(capabilities.minImageExtent.height, std::min(capabilities.maxImageExtent.height, actualExtent.height));

    return actualExtent;
}

void Renderer::create_swapchain()
{
    device->waitIdle();

    swapchain_data = {};

    auto surface_caps = physical_device.getSurfaceCapabilitiesKHR(display_surface.get());
    auto format = chose_surface_format(physical_device, display_surface.get());
    auto present_mode = chose_present_mode(physical_device, display_surface.get());
    auto extent = chose_swap_extent(surface_caps);

    uint32_t image_count = surface_caps.minImageCount + 1;

    if (surface_caps.maxImageCount > 0)
    {
        image_count = std::min(image_count, surface_caps.maxImageCount);
    }

    bool queues_differ =
        queue_family_indices.graphics_queue_idx != queue_family_indices.presentation_queue_idx;

    std::vector<uint32_t> queue_families;
    if (queues_differ)
    {
        queue_families = {queue_family_indices.graphics_queue_idx, queue_family_indices.presentation_queue_idx};
    }

    swapchain_data.swapchain = device->createSwapchainKHRUnique(vk::SwapchainCreateInfoKHR{
        {}, display_surface.get(),
        image_count, format.format, format.colorSpace,
        extent, 1, vk::ImageUsageFlagBits::eColorAttachment,
        queues_differ ? vk::SharingMode::eConcurrent : vk::SharingMode::eExclusive,
        static_cast<uint32_t>(queue_families.size()), queue_families.data(),
        surface_caps.currentTransform, vk::CompositeAlphaFlagBitsKHR::eOpaque,
        present_mode,
        /* clipped */ true,
        /* old swapchain */ nullptr
    });

    swapchain_data.elements.clear();

    auto imgs = device->getSwapchainImagesKHR(swapchain_data.swapchain.get());
    swapchain_data.elements.reserve(imgs.size());
    for (auto& img : imgs)
    {
        swapchain_data.elements.emplace_back(SwapchainData::PerElementData{img});
    }

    swapchain_data.format = format.format;
    swapchain_data.extent = extent;

    vk::AttachmentDescription attachment_description{
        {}, swapchain_data.format, vk::SampleCountFlagBits::e1,
        /* attachment ops */vk::AttachmentLoadOp::eClear, vk::AttachmentStoreOp::eStore,
        /* stencil ops */ vk::AttachmentLoadOp::eDontCare, vk::AttachmentStoreOp::eDontCare,
        vk::ImageLayout::eUndefined, vk::ImageLayout::ePresentSrcKHR
    };

    vk::AttachmentReference attachment_reference{
        /* attachment */ 0,
                         vk::ImageLayout::eColorAttachmentOptimal
    };

    vk::SubpassDescription subpass_description{
        {}, vk::PipelineBindPoint::eGraphics,
        /* input attachments */ 0, nullptr,
        /* color attachments */ 1, &attachment_reference
    };

    vk::SubpassDependency dependency{
        /* src */ VK_SUBPASS_EXTERNAL,
        /* dst */ 0,
        /* src stages */ vk::PipelineStageFlagBits::eColorAttachmentOutput,
        /* dst stages */ vk::PipelineStageFlagBits::eColorAttachmentOutput,
        /* src access */ {},
        /* dst access */ vk::AccessFlagBits::eColorAttachmentWrite
    };

    swapchain_data.render_pass = device->createRenderPassUnique(vk::RenderPassCreateInfo{
        {},
        1, &attachment_description,
        1, &subpass_description,
        1, &dependency
    });




    for (auto& el : swapchain_data.elements)
    {
        el.image_view = device->createImageViewUnique(vk::ImageViewCreateInfo{
            {}, el.image, vk::ImageViewType::e2D, format.format,
            vk::ComponentMapping{},
            vk::ImageSubresourceRange{vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1}
        });

        std::array attachments {el.image_view.get()};

        el.framebuffer = device->createFramebufferUnique(vk::FramebufferCreateInfo{
            {}, swapchain_data.render_pass.get(),
            static_cast<uint32_t>(attachments.size()), attachments.data(),
            swapchain_data.extent.width, swapchain_data.extent.height,
            /* layers */ 1u
        });
    }

    auto cmd_buffers = device->allocateCommandBuffersUnique(vk::CommandBufferAllocateInfo{
        command_pool.get(), vk::CommandBufferLevel::ePrimary, static_cast<uint32_t>(swapchain_data.elements.size())
    });

    for (std::size_t i = 0; i < swapchain_data.elements.size(); ++i)
    {
        swapchain_data.elements[i].command_buffer = std::move(cmd_buffers[i]);
    }

    if (scene != nullptr)
    {
        scene->recreate_pipelines(PipelineCreationInfo{
            swapchain_data.render_pass.get(),
            swapchain_data.extent
        });
        // TODO: remove
        for (std::size_t i = 0; i < swapchain_data.elements.size(); ++i)
        {
            record_commands(i);
        }
    }
}

void Renderer::on_window_resized()
{
    swapchain_data.needs_recreation = true;
}

void Renderer::record_commands(std::size_t swapchain_idx)
{
    auto& swapchain_element = swapchain_data.elements[swapchain_idx];
    auto& cb = swapchain_element.command_buffer;
    cb->begin(vk::CommandBufferBeginInfo{});
    {
        std::array clear_colors{vk::ClearValue{std::array{0.f, 0.f, 0.f, 1.f}}};

        cb->beginRenderPass(vk::RenderPassBeginInfo{
            swapchain_data.render_pass.get(),
            swapchain_element.framebuffer.get(),
            vk::Rect2D{{0, 0}, swapchain_data.extent},
            static_cast<uint32_t>(clear_colors.size()), clear_colors.data()
        }, vk::SubpassContents::eInline);

        {
            scene->record_commands(cb.get());
        }

        cb->endRenderPass();
    }
    cb->end();
}

Renderer::~Renderer()
{
    device->waitIdle();
}

RingBuffer Renderer::create_ubo(std::size_t size)
{
    return RingBuffer({
        allocator, MAX_FRAMES_IN_FLIGHT, size,
        vk::BufferUsageFlagBits::eUniformBuffer,
        VMA_MEMORY_USAGE_CPU_TO_GPU
    });
}

UniformRing Renderer::create_descriptor_set(vk::DescriptorSetLayout layout)
{
    return UniformRing({
        device.get(), global_descriptor_pool.get(), MAX_FRAMES_IN_FLIGHT, layout
    });
}

UniqueVmaBuffer Renderer::create_vbo(std::size_t size)
{
    return UniqueVmaBuffer(allocator, size, vk::BufferUsageFlagBits::eVertexBuffer, VMA_MEMORY_USAGE_CPU_TO_GPU);
}
