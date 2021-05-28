#pragma once

#include <vulkan/vulkan.hpp>
#include <function2/function2.hpp>
#include <span>
#include <vk_mem_alloc.h>

#include "Scene.hpp"
#include "ResourceManager.hpp"
#include "UniqueVmaImage.hpp"
#include "Gui.hpp"
#include "data_primitives/RingBuffer.hpp"
#include "data_primitives/DescriptorSetRing.hpp"


struct QueueFamilyIndices
{
    uint32_t graphics_queue_idx{};
    uint32_t presentation_queue_idx{};
};

class Renderer : public IResourceManager {
    static constexpr std::size_t MAX_FRAMES_IN_FLIGHT = 2;
public:
    /**
     * All extension and validation layer management is located inside of Application.cpp for simplicity's sake
     * @param instance
     * @param validation_layers
     * @param extensions
     */
    explicit Renderer(vk::Instance instance, vk::UniqueSurfaceKHR surface,
        std::span<const char* const> validation_layers, fu2::unique_function<vk::Extent2D()> res_provider);

    void render(float delta_seconds);

    void on_window_resized();

    vk::Device get_device() override { return device_.get(); };

    Scene* debug_get_scene() { return scene_.get(); }

    // begin IResourceManager

    RingBuffer create_ubo(std::size_t size) override;
    RingBuffer create_sbo(std::size_t size) override;
    DescriptorSetRing create_descriptor_set_ring(vk::DescriptorSetLayout layout) override;
    vk::UniqueDescriptorSet create_descriptor_set(vk::DescriptorSetLayout layout) override;
    UniqueVmaBuffer create_vbo(std::size_t size) override;
    RingBuffer create_dynamic_vbo(std::size_t size) override;
    UniqueVmaImage create_texture(vk::Extent2D extent, std::size_t layers) override;
    UniqueVmaBuffer create_staging_buffer(std::size_t size) override;

    VirtualTextureSet create_svt(std::size_t gpu_cache_side_size, std::size_t per_frame_update_limit, vk::Format format,
        std::size_t format_multiplicity, std::size_t min_mip,
        std::vector<std::vector<const std::byte*>> image_mip_data) override;

    vk::UniqueCommandBuffer begin_single_time_commands() override;
    void finish_single_time_commands(vk::UniqueCommandBuffer cb) override;

    // end IResourceManager

    void hotswap_shaders();


    ~Renderer() override;

private:
    vk::Extent2D chose_swap_extent(const vk::SurfaceCapabilitiesKHR& capabilities);

    // TODO: this procedural-like initialization pattern is questionable
    void create_swapchain();

    void create_gui_framebuffers();

    void record_commands(std::size_t swapchain_idx);
private:
    constexpr static auto DEPTHBUFFER_FORMAT = vk::Format::eD32Sfloat;

private:
    fu2::unique_function<vk::Extent2D()> resolution_provider_;

    vk::Instance vulkan_instance_;
    vk::UniqueSurfaceKHR display_surface_;
    vk::PhysicalDevice physical_device_;

    QueueFamilyIndices queue_family_indices_;
    vk::UniqueDevice device_;
    vk::Queue graphics_queue_;
    vk::Queue present_queue_;

    VmaAllocator allocator_{nullptr};
    std::unique_ptr<void, void(*)(void*)> deferred_allocator_destroy_{nullptr, nullptr};

    vk::UniqueDescriptorPool global_descriptor_pool_;
    vk::UniqueCommandPool single_use_command_pool_;

    struct SwapchainData
    {
        bool needs_recreation{false};
        vk::UniqueSwapchainKHR swapchain;
        vk::Format format;
        vk::Extent2D extent;

        UniqueVmaImage depthbuffer;
        vk::UniqueImageView depthbuffer_view;

        vk::UniqueRenderPass render_pass;

        struct PerElementData
        {
            vk::Image image;
            vk::UniqueImageView image_view;
            vk::Fence image_fence;
            vk::UniqueFramebuffer main_framebuffer;
            vk::UniqueFramebuffer gui_framebuffer;

            vk::UniqueCommandPool command_pool;
            vk::UniqueCommandBuffer command_buffer;
        };

        std::vector<PerElementData> elements;
    };

    SwapchainData swapchain_data_;

    std::unique_ptr<Scene> scene_;
    std::unique_ptr<Gui> gui_;

    std::size_t current_frame_idx_{0};

    struct PerInflightFrameData
    {
        vk::UniqueSemaphore image_available_semaphore;
        vk::UniqueSemaphore rendering_finished_semaphore;
        vk::UniqueFence in_flight_fence;
    };

    std::array<PerInflightFrameData, MAX_FRAMES_IN_FLIGHT> per_inflight_frame_data_;
};
