#pragma once

#include <vulkan/vulkan.hpp>
#include <backends/imgui_impl_vulkan.h>


// https://frguthmann.github.io/posts/vulkan_imgui/
class Gui
{
public:
    struct CreateInfo
    {
        vk::Instance instance;
        vk::PhysicalDevice physical_device;
        vk::Device device;
        class IResourceManager* irm;

        uint32_t graphics_queue_idx;
        vk::Queue graphics_queue;
        std::size_t swapchain_size;
        vk::Format format;
    };

    explicit Gui(CreateInfo info);

    [[nodiscard]] vk::RenderPass get_render_pass() const { return render_pass.get(); };

    void tick();

    void render();

    void record_commands(vk::CommandBuffer cb, vk::Framebuffer framebuffer, vk::Rect2D area);
private:
    vk::UniqueDescriptorPool descriptor_pool;
    vk::UniqueRenderPass render_pass;

};
