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

Renderer::Renderer(vk::Instance instance, vk::UniqueSurfaceKHR surface,
    std::span<const char* const> validation_layers,
    fu2::unique_function<vk::Extent2D()> res_provider)
    : resolution_provider_{std::move(res_provider)}
    , vulkan_instance_{instance}
    , display_surface_{std::move(surface)}
{
    // TODO: Proper device selection
    physical_device_ = vulkan_instance_.enumeratePhysicalDevices().front();

    queue_family_indices_ = chose_queue_families(physical_device_, display_surface_.get());    

    // kostyl due to bad API design
    float queue_priority = 1.0f;
    std::vector<vk::DeviceQueueCreateInfo> queue_create_infos;
    {
        std::unordered_set<uint32_t> unique_families{
            queue_family_indices_.graphics_queue_idx,
            queue_family_indices_.presentation_queue_idx
        };

        queue_create_infos.reserve(unique_families.size());
        for (auto idx : unique_families)
        {
            queue_create_infos.emplace_back(vk::DeviceQueueCreateInfo{{}, idx, 1, &queue_priority});
        }
    }


    vk::PhysicalDeviceFeatures features;

    features.tessellationShader = true;
    features.geometryShader = true;

    // NOTE: newer vulkan implementations ignore the layers here
    device_ = physical_device_.createDeviceUnique(vk::DeviceCreateInfo{
        {},
        static_cast<uint32_t>(queue_create_infos.size()), queue_create_infos.data(),
        static_cast<uint32_t>(validation_layers.size()), validation_layers.data(),
        static_cast<uint32_t>(DEVICE_EXTENSIONS.size()), DEVICE_EXTENSIONS.data(),
        &features
    });
    
    graphics_queue_ = device_->getQueue(queue_family_indices_.graphics_queue_idx, 0);
    present_queue_ = device_->getQueue(queue_family_indices_.presentation_queue_idx, 0);

    {
        VmaAllocatorCreateInfo info{
            .flags = {},
            .physicalDevice = physical_device_,
            .device = device_.get(),

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
        vmaCreateAllocator(&info, &allocator_);

        deferred_allocator_destroy_ =
            decltype(deferred_allocator_destroy_)(&allocator_,
                [](void* alloc)
                {
                    vmaDestroyAllocator(*reinterpret_cast<VmaAllocator*>(alloc));
                });
    }

    create_swapchain();

    {
        // TODO: This really confuses me...
        std::size_t max_objects = 100;
        // 2 sets: one global, one per-model
        std::size_t max_sets = MAX_FRAMES_IN_FLIGHT * (max_objects + 1);
        std::array pool_sizes{
            vk::DescriptorPoolSize
                {vk::DescriptorType::eUniformBuffer, static_cast<uint32_t>(max_objects * max_sets)},
            vk::DescriptorPoolSize
                {vk::DescriptorType::eCombinedImageSampler, static_cast<uint32_t>(max_objects * max_sets)},
            vk::DescriptorPoolSize
                {vk::DescriptorType::eStorageBuffer, static_cast<uint32_t>(max_objects * max_sets)}
        };
        global_descriptor_pool_ = device_->createDescriptorPoolUnique(vk::DescriptorPoolCreateInfo{
            {},
            /* max sets */ static_cast<uint32_t>(max_sets),
            /* pool sizes */ static_cast<uint32_t>(pool_sizes.size()), pool_sizes.data()
        });
    }

    single_use_command_pool_ = device_->createCommandPoolUnique(vk::CommandPoolCreateInfo{
        vk::CommandPoolCreateFlagBits::eResetCommandBuffer, queue_family_indices_.graphics_queue_idx
    });

    scene_ = std::make_unique<Scene>(this, PipelineCreationInfo{
        swapchain_data_.render_pass.get(),
        swapchain_data_.extent
    });

    gui_ = std::make_unique<Gui>(Gui::CreateInfo{
        instance, physical_device_, device_.get(), this,
        queue_family_indices_.graphics_queue_idx, graphics_queue_,
        swapchain_data_.elements.size(), swapchain_data_.format
    });

    create_gui_framebuffers();

    for (auto& data : per_inflight_frame_data_)
    {
        data.image_available_semaphore = device_->createSemaphoreUnique(vk::SemaphoreCreateInfo{});
        data.rendering_finished_semaphore = device_->createSemaphoreUnique(vk::SemaphoreCreateInfo{});
        data.in_flight_fence = device_->createFenceUnique(vk::FenceCreateInfo{vk::FenceCreateFlagBits::eSignaled});
    }
}

void Renderer::render(float delta_seconds)
{
    auto& data = per_inflight_frame_data_[current_frame_idx_++ % MAX_FRAMES_IN_FLIGHT];

    if (device_->waitForFences({data.in_flight_fence.get()}, true, std::numeric_limits<uint64_t>::max())
        != vk::Result::eSuccess)
    {
        throw std::runtime_error("Inflight fence timeout!");
    }

    uint32_t idx;
    {
        auto result = device_->acquireNextImageKHR(swapchain_data_.swapchain.get(),
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

    // Cannot be moved further up as recreating the swapchain without running record_commands is wrong!!!
    // tick creates objects, record_commands consumes. If objects don't get consumed, everything breaks
    gui_->tick(delta_seconds);
    // Scene::tick can show debug imgui thingies
    scene_->tick(delta_seconds);
    gui_->render();

    auto& swapchain_element = swapchain_data_.elements[idx];

    if (swapchain_element.image_fence)
    {
        if (device_->waitForFences({swapchain_element.image_fence}, true, std::numeric_limits<uint64_t>::max())
            != vk::Result::eSuccess)
        {
            throw std::runtime_error("Swapchain image fence timeout!");
        }
    }

    swapchain_element.image_fence = data.in_flight_fence.get();

    device_->resetCommandPool(swapchain_element.command_pool.get(), vk::CommandPoolResetFlagBits{});
    record_commands(idx);


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

    device_->resetFences({data.in_flight_fence.get()});

    if (graphics_queue_.submit(
        static_cast<uint32_t>(submits.size()), submits.data(), data.in_flight_fence.get()) != vk::Result::eSuccess)
    {
        throw std::runtime_error("Graphics queue submission failed!");
    }

    {
        std::array swapchains{swapchain_data_.swapchain.get()};

        vk::PresentInfoKHR present_info{
            static_cast<uint32_t>(signal_semos.size()), signal_semos.data(),
            static_cast<uint32_t>(swapchains.size()), swapchains.data(),
            &idx
        };

        auto result = present_queue_.presentKHR(&present_info);

        if (result == vk::Result::eErrorOutOfDateKHR || result == vk::Result::eSuboptimalKHR
            || swapchain_data_.needs_recreation)
        {
            create_swapchain();
        }
        else if (result != vk::Result::eSuccess)
        {
            throw std::runtime_error("Graphics queue submission failed!");
        }
    }
}

vk::Extent2D Renderer::chose_swap_extent(const vk::SurfaceCapabilitiesKHR& capabilities)
{
    if (capabilities.currentExtent.width != std::numeric_limits<uint32_t>::max()) {
        return capabilities.currentExtent;
    }

    VkExtent2D actualExtent = resolution_provider_();

    actualExtent.width = std::max(capabilities.minImageExtent.width, std::min(capabilities.maxImageExtent.width, actualExtent.width));
    actualExtent.height = std::max(capabilities.minImageExtent.height, std::min(capabilities.maxImageExtent.height, actualExtent.height));

    return actualExtent;
}

void Renderer::create_swapchain()
{
    device_->waitIdle();

    swapchain_data_ = {};

    auto surface_caps = physical_device_.getSurfaceCapabilitiesKHR(display_surface_.get());
    auto format = chose_surface_format(physical_device_, display_surface_.get());
    auto present_mode = chose_present_mode(physical_device_, display_surface_.get());
    auto extent = chose_swap_extent(surface_caps);

    uint32_t image_count = surface_caps.minImageCount + 1;

    if (surface_caps.maxImageCount > 0)
    {
        image_count = std::min(image_count, surface_caps.maxImageCount);
    }

    bool queues_differ =
        queue_family_indices_.graphics_queue_idx != queue_family_indices_.presentation_queue_idx;

    std::vector<uint32_t> queue_families;
    if (queues_differ)
    {
        queue_families = {queue_family_indices_.graphics_queue_idx, queue_family_indices_.presentation_queue_idx};
    }

    swapchain_data_.swapchain = device_->createSwapchainKHRUnique(vk::SwapchainCreateInfoKHR{
        {}, display_surface_.get(),
        image_count, format.format, format.colorSpace,
        extent, 1, vk::ImageUsageFlagBits::eColorAttachment,
        queues_differ ? vk::SharingMode::eConcurrent : vk::SharingMode::eExclusive,
        static_cast<uint32_t>(queue_families.size()), queue_families.data(),
        surface_caps.currentTransform, vk::CompositeAlphaFlagBitsKHR::eOpaque,
        present_mode,
        /* clipped */ true,
        /* old swapchain */ swapchain_data_.swapchain.get()
    });

    swapchain_data_.elements.clear();

    auto imgs = device_->getSwapchainImagesKHR(swapchain_data_.swapchain.get());
    swapchain_data_.elements.reserve(imgs.size());
    for (auto& img : imgs)
    {
        swapchain_data_.elements.emplace_back(SwapchainData::PerElementData{img});
    }

    swapchain_data_.format = format.format;
    swapchain_data_.extent = extent;



    swapchain_data_.depthbuffer = UniqueVmaImage(allocator_, DEPTHBUFFER_FORMAT, swapchain_data_.extent, vk::ImageTiling::eOptimal,
        vk::ImageUsageFlagBits::eDepthStencilAttachment, VMA_MEMORY_USAGE_GPU_ONLY);

    swapchain_data_.depthbuffer_view = device_->createImageViewUnique(vk::ImageViewCreateInfo{
        {}, swapchain_data_.depthbuffer.get(), vk::ImageViewType::e2D, DEPTHBUFFER_FORMAT,
        vk::ComponentMapping{},
        vk::ImageSubresourceRange{vk::ImageAspectFlagBits::eDepth, 0, 1, 0, 1}
    });


    std::array attachment_descriptions{
        vk::AttachmentDescription{
            {}, swapchain_data_.format, vk::SampleCountFlagBits::e1,
            /* attachment ops */vk::AttachmentLoadOp::eClear, vk::AttachmentStoreOp::eStore,
            /* stencil ops */ vk::AttachmentLoadOp::eDontCare, vk::AttachmentStoreOp::eDontCare,
            vk::ImageLayout::eUndefined, vk::ImageLayout::eColorAttachmentOptimal
        },
        vk::AttachmentDescription{
            {}, DEPTHBUFFER_FORMAT, vk::SampleCountFlagBits::e1,
            /* attachment ops */ vk::AttachmentLoadOp::eClear, vk::AttachmentStoreOp::eDontCare,
            /* stencil ops */ vk::AttachmentLoadOp::eDontCare, vk::AttachmentStoreOp::eDontCare,
            vk::ImageLayout::eUndefined, vk::ImageLayout::eDepthStencilAttachmentOptimal
        }
   };

    vk::AttachmentReference color_attachment_reference{
        /* attachment */ 0,
        vk::ImageLayout::eColorAttachmentOptimal
    };

    vk::AttachmentReference depth_attachment_reference{
        /* attachment */ 1,
        vk::ImageLayout::eDepthStencilAttachmentOptimal
    };

    vk::SubpassDescription subpass_description{
        {}, vk::PipelineBindPoint::eGraphics,
        /* input */ 0, nullptr,
        /* attach count */ 1,
        /* color */         &color_attachment_reference,
        /* resolve */       nullptr,
        /* depth/stencil */ &depth_attachment_reference
    };

    std::array dependencies{
        vk::SubpassDependency{
            /* src */ VK_SUBPASS_EXTERNAL,
            /* dst */ 0,
            /* src stages */
            vk::PipelineStageFlagBits::eColorAttachmentOutput | vk::PipelineStageFlagBits::eEarlyFragmentTests,
            /* dst stages */
            vk::PipelineStageFlagBits::eColorAttachmentOutput | vk::PipelineStageFlagBits::eEarlyFragmentTests,
            /* src access */ {},
            /* dst access */ vk::AccessFlagBits::eColorAttachmentWrite | vk::AccessFlagBits::eDepthStencilAttachmentWrite
        }
    };

    swapchain_data_.render_pass = device_->createRenderPassUnique(vk::RenderPassCreateInfo{
        {},
        static_cast<uint32_t>(attachment_descriptions.size()), attachment_descriptions.data(),
        1, &subpass_description,
        static_cast<uint32_t>(dependencies.size()), dependencies.data()
    });




    for (auto& el : swapchain_data_.elements)
    {
        el.image_view = device_->createImageViewUnique(vk::ImageViewCreateInfo{
            {}, el.image, vk::ImageViewType::e2D, format.format,
            vk::ComponentMapping{},
            vk::ImageSubresourceRange{vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1}
        });

        std::array attachments{el.image_view.get(), swapchain_data_.depthbuffer_view.get()};

        el.main_framebuffer = device_->createFramebufferUnique(vk::FramebufferCreateInfo{
            {}, swapchain_data_.render_pass.get(),
            static_cast<uint32_t>(attachments.size()), attachments.data(),
            swapchain_data_.extent.width, swapchain_data_.extent.height,
            /* layers */ 1u
        });
    }


    for (auto& element : swapchain_data_.elements)
    {
        // as per https://developer.nvidia.com/blog/vulkan-dos-donts/
        element.command_pool = device_->createCommandPoolUnique(vk::CommandPoolCreateInfo{
            {}, queue_family_indices_.graphics_queue_idx
        });
    }

    for (auto& element : swapchain_data_.elements)
    {
        element.command_buffer =
            std::move(device_->allocateCommandBuffersUnique(vk::CommandBufferAllocateInfo{
                element.command_pool.get(),
                vk::CommandBufferLevel::ePrimary, 1
            }).front());
    }

    if (scene_ != nullptr)
    {
        scene_->recreate_pipelines(PipelineCreationInfo{
            swapchain_data_.render_pass.get(),
            swapchain_data_.extent
        });
    }

    if (gui_ != nullptr)
    {
        create_gui_framebuffers();
    }
}

void Renderer::create_gui_framebuffers()
{
    // TODO: consider moving this to Gui.cpp
    for (auto& el : swapchain_data_.elements)
    {
        std::array attachments{el.image_view.get()};

        el.gui_framebuffer = device_->createFramebufferUnique(vk::FramebufferCreateInfo{
            {}, gui_->get_render_pass(),
            static_cast<uint32_t>(attachments.size()), attachments.data(),
            swapchain_data_.extent.width, swapchain_data_.extent.height,
            /* layers */ 1u
        });
    }
}

void Renderer::on_window_resized()
{
    swapchain_data_.needs_recreation = true;
}

void Renderer::record_commands(std::size_t swapchain_idx)
{
    auto& swapchain_element = swapchain_data_.elements[swapchain_idx];
    auto cb = swapchain_element.command_buffer.get();

    cb.begin(vk::CommandBufferBeginInfo{vk::CommandBufferUsageFlagBits::eOneTimeSubmit});
    {
        scene_->record_pre_commands(cb);

        std::array clear_colors{
            vk::ClearValue{vk::ClearColorValue{std::array{0.f, 0.f, 0.f, 1.f}}},
            vk::ClearValue{vk::ClearDepthStencilValue{1, 0}}
        };

        vk::Rect2D area{{0, 0}, swapchain_data_.extent};

        cb.beginRenderPass(vk::RenderPassBeginInfo{
            swapchain_data_.render_pass.get(),
            swapchain_element.main_framebuffer.get(),
            area,
            static_cast<uint32_t>(clear_colors.size()), clear_colors.data()
        }, vk::SubpassContents::eInline);

        {
            scene_->record_commands(cb);
        }

        cb.endRenderPass();

        gui_->record_commands(cb, swapchain_element.gui_framebuffer.get(), area);
    }
    cb.end();
}

Renderer::~Renderer()
{
    device_->waitIdle();

    ImGui_ImplVulkan_Shutdown();
}

RingBuffer Renderer::create_ubo(std::size_t size)
{
    return RingBuffer({
        allocator_, MAX_FRAMES_IN_FLIGHT, size,
        vk::BufferUsageFlagBits::eUniformBuffer,
        VMA_MEMORY_USAGE_CPU_TO_GPU
    });
}

RingBuffer Renderer::create_sbo(std::size_t size)
{
    return RingBuffer({
        allocator_, MAX_FRAMES_IN_FLIGHT, size,
        vk::BufferUsageFlagBits::eStorageBuffer,
        VMA_MEMORY_USAGE_CPU_TO_GPU
    });
}

DescriptorSetRing Renderer::create_descriptor_set_ring(vk::DescriptorSetLayout layout)
{
    return DescriptorSetRing({
        device_.get(), global_descriptor_pool_.get(), MAX_FRAMES_IN_FLIGHT, layout
    });
}

vk::UniqueDescriptorSet Renderer::create_descriptor_set(vk::DescriptorSetLayout layout)
{
    return std::move(device_->allocateDescriptorSetsUnique(vk::DescriptorSetAllocateInfo{
        global_descriptor_pool_.get(), 1, &layout
    }).front());
}

UniqueVmaBuffer Renderer::create_vbo(std::size_t size)
{
    return UniqueVmaBuffer(allocator_, size, vk::BufferUsageFlagBits::eVertexBuffer, VMA_MEMORY_USAGE_CPU_TO_GPU);
}

UniqueVmaBuffer Renderer::create_ibo(std::size_t size)
{
    return UniqueVmaBuffer(allocator_, size, vk::BufferUsageFlagBits::eIndexBuffer, VMA_MEMORY_USAGE_CPU_TO_GPU);
}

RingBuffer Renderer::create_dynamic_vbo(std::size_t size)
{
    return RingBuffer({
        allocator_, MAX_FRAMES_IN_FLIGHT, size,
        vk::BufferUsageFlagBits::eVertexBuffer,
        VMA_MEMORY_USAGE_CPU_TO_GPU
    });
}

UniqueVmaImage Renderer::create_texture(vk::Extent2D extent, std::size_t layers)
{
    return UniqueVmaImage(allocator_, vk::Format::eR8G8B8A8Srgb, extent, vk::ImageTiling::eOptimal,
        vk::ImageUsageFlagBits::eTransferDst | vk::ImageUsageFlagBits::eSampled, VMA_MEMORY_USAGE_GPU_ONLY, layers);
}

UniqueVmaBuffer Renderer::create_staging_buffer(std::size_t size)
{
    return UniqueVmaBuffer(allocator_, size, vk::BufferUsageFlagBits::eTransferSrc, VMA_MEMORY_USAGE_CPU_TO_GPU);
}

VirtualTextureSet
Renderer::create_svt(std::size_t gpu_cache_side_size, std::size_t per_frame_update_limit, vk::Format format,
    std::size_t format_multiplicity, std::size_t min_mip, std::vector<std::vector<const std::byte*>> image_mip_data)
{
    auto cb = begin_single_time_commands();
    VirtualTextureSet result(VirtualTextureSet::CreateInfo{
        device_.get(),
        allocator_, cb.get(), MAX_FRAMES_IN_FLIGHT,
        gpu_cache_side_size, per_frame_update_limit, format, format_multiplicity,
        min_mip, std::move(image_mip_data)
    });
    finish_single_time_commands(std::move(cb));

    return result;
}

vk::UniqueCommandBuffer Renderer::begin_single_time_commands()
{
    vk::UniqueCommandBuffer result = std::move(device_->allocateCommandBuffersUnique(vk::CommandBufferAllocateInfo{
        single_use_command_pool_.get(), vk::CommandBufferLevel::ePrimary, 1
    }).front());

    result->begin(vk::CommandBufferBeginInfo{vk::CommandBufferUsageFlagBits::eOneTimeSubmit});

    return result;
}

void Renderer::finish_single_time_commands(vk::UniqueCommandBuffer cb)
{
    cb->end();

    graphics_queue_.submit({vk::SubmitInfo{
        /* wait semos */ 0, nullptr, nullptr,
        1, &cb.get()
    }}, nullptr);
    // TODO: actual fences
    graphics_queue_.waitIdle();
}

void Renderer::hotswap_shaders()
{
    scene_->reload_shaders();
    device_->waitIdle();
    scene_->recreate_pipelines(PipelineCreationInfo{
        swapchain_data_.render_pass.get(),
        swapchain_data_.extent
    });
}
