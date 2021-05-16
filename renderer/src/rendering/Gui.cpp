#include "Gui.hpp"

#include <backends/imgui_impl_glfw.h>

#include "ResourceManager.hpp"


Gui::Gui(CreateInfo info)
{
    std::array pool_sizes{
        vk::DescriptorPoolSize{vk::DescriptorType::eSampler, 1000},
        vk::DescriptorPoolSize{vk::DescriptorType::eCombinedImageSampler, 1000},
        vk::DescriptorPoolSize{vk::DescriptorType::eSampledImage, 1000},
        vk::DescriptorPoolSize{vk::DescriptorType::eStorageImage, 1000},
        vk::DescriptorPoolSize{vk::DescriptorType::eUniformTexelBuffer, 1000},
        vk::DescriptorPoolSize{vk::DescriptorType::eStorageTexelBuffer, 1000},
        vk::DescriptorPoolSize{vk::DescriptorType::eUniformBuffer, 1000},
        vk::DescriptorPoolSize{vk::DescriptorType::eStorageBuffer, 1000},
        vk::DescriptorPoolSize{vk::DescriptorType::eUniformBufferDynamic, 1000},
        vk::DescriptorPoolSize{vk::DescriptorType::eStorageBufferDynamic, 1000},
        vk::DescriptorPoolSize{vk::DescriptorType::eInputAttachment, 1000},
    };
    descriptor_pool = info.device.createDescriptorPoolUnique(vk::DescriptorPoolCreateInfo{
        {},
        /* max sets */ 1000 * static_cast<uint32_t>(pool_sizes.size()),
        /* pool sizes */ static_cast<uint32_t>(pool_sizes.size()), pool_sizes.data()
    });

    std::array attachment_descriptions{
        vk::AttachmentDescription{
            {}, info.format, vk::SampleCountFlagBits::e1,
            /* attachment ops */vk::AttachmentLoadOp::eLoad, vk::AttachmentStoreOp::eStore,
            /* stencil ops */ vk::AttachmentLoadOp::eDontCare, vk::AttachmentStoreOp::eDontCare,
            vk::ImageLayout::eColorAttachmentOptimal, vk::ImageLayout::ePresentSrcKHR
        }
    };

    vk::AttachmentReference color_attachment_reference{
        /* attachment */ 0,
        vk::ImageLayout::eColorAttachmentOptimal
    };

    vk::SubpassDescription subpass_description{
        {}, vk::PipelineBindPoint::eGraphics,
        /* input */ 0, nullptr,
        /* color */ 1, &color_attachment_reference,
    };

    vk::SubpassDependency dependency{
        /* src */ VK_SUBPASS_EXTERNAL,
        /* dst */ 0,
        /* src stages */
                  vk::PipelineStageFlagBits::eColorAttachmentOutput,
        /* dst stages */
                  vk::PipelineStageFlagBits::eColorAttachmentOutput,
        /* src access */ {},
        /* dst access */ vk::AccessFlagBits::eColorAttachmentWrite
    };

    render_pass = info.device.createRenderPassUnique(vk::RenderPassCreateInfo{
        {},
        static_cast<uint32_t>(attachment_descriptions.size()), attachment_descriptions.data(),
        1, &subpass_description,
        1, &dependency
    });


    ImGui_ImplVulkan_InitInfo init_info{
        .Instance = info.instance,
        .PhysicalDevice = info.physical_device,
        .Device = info.device,
        .QueueFamily = info.graphics_queue_idx,
        .Queue = info.graphics_queue,
        .PipelineCache = VK_NULL_HANDLE,
        .DescriptorPool = descriptor_pool.get(),
        .MinImageCount = 2,
        .ImageCount = static_cast<uint32_t>(info.swapchain_size),
        .MSAASamples = VkSampleCountFlagBits::VK_SAMPLE_COUNT_1_BIT,
        .Allocator = nullptr,
        .CheckVkResultFn = [](VkResult res)
        {
            if (res != VK_SUCCESS)
            {
                throw std::runtime_error("dear imgui made a poo poo");
            }
        },
    };

    ImGui_ImplVulkan_Init(&init_info, render_pass.get());

    auto cb = info.irm->begin_single_time_commands();
    ImGui_ImplVulkan_CreateFontsTexture(cb.get());
    info.irm->finish_single_time_commands(std::move(cb));

    ImGui_ImplVulkan_DestroyFontUploadObjects();
}

void Gui::tick(float delta_seconds)
{
    ImGui_ImplVulkan_NewFrame();
    ImGui::NewFrame();

    ImGui::Text("%f fps", 1.f/delta_seconds);
    ImGui::Text("%f ms", delta_seconds * 1000.f);
}

void Gui::render()
{
    ImGui::Render();
}

void Gui::record_commands(vk::CommandBuffer cb, vk::Framebuffer framebuffer, vk::Rect2D area)
{
    std::array clear_colors{
        vk::ClearValue{vk::ClearColorValue{std::array{0.f, 0.f, 0.f, 1.f}}}
    };

    cb.beginRenderPass(vk::RenderPassBeginInfo{
        render_pass.get(), framebuffer, area,
        static_cast<uint32_t>(clear_colors.size()), clear_colors.data()
    }, vk::SubpassContents::eInline);

    ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), cb);

    cb.endRenderPass();
}
