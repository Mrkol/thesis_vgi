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
        class IResourceManager* irm{nullptr};

        uint32_t graphics_queue_idx{};
        vk::Queue graphics_queue;
        std::size_t swapchain_size{};
        vk::Format format{};
    };

    explicit Gui(CreateInfo info);

    [[nodiscard]] vk::RenderPass get_render_pass() const { return render_pass_.get(); };

    void tick(float delta_seconds);

    void render();

    void record_commands(vk::CommandBuffer cb, vk::Framebuffer framebuffer, vk::Rect2D area);

public:
    bool show_gui{true};

private:
    vk::UniqueDescriptorPool descriptor_pool_;
    vk::UniqueRenderPass render_pass_;

    std::array<float, 1000> last_frame_ms_{1};
    std::size_t last_frame_idx_{0};
};
