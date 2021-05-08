#pragma once

#include <vulkan/vulkan.hpp>
#include <function2/function2.hpp>
#include <span>
#include <vk_mem_alloc.h>

#include "Scene.hpp"
#include "ResourceManager.hpp"


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
    explicit Renderer(vk::Instance& instance, vk::UniqueSurfaceKHR surface,
        std::span<const char* const> validation_layers, fu2::unique_function<vk::Extent2D()> res_provider);

    void render();

    void on_window_resized();

    vk::Device get_device() override { return device.get(); };

    Scene* debug_get_scene() { return scene.get(); }

    RingBuffer create_ubo(std::size_t size) override;
    UniformRing create_descriptor_set(vk::DescriptorSetLayout layout) override;
    UniqueVmaBuffer create_vbo(std::size_t size) override;

    ~Renderer() override;

private:
    vk::Extent2D chose_swap_extent(const vk::SurfaceCapabilitiesKHR& capabilities);

    // TODO: this procedural-like initialization pattern is questionable
    void create_swapchain();

    void record_commands(std::size_t swapchain_idx);

private:
    fu2::unique_function<vk::Extent2D()> resolution_provider;

    vk::Instance* vulkan_instance;
    vk::UniqueSurfaceKHR display_surface;
    vk::PhysicalDevice physical_device;

    QueueFamilyIndices queue_family_indices;
    vk::UniqueDevice device;
    vk::Queue graphics_queue;
    vk::Queue present_queue;

    VmaAllocator allocator{VK_NULL_HANDLE};
    std::unique_ptr<void, void(*)(void*)> deferred_allocator_destroy{nullptr, nullptr};

    vk::UniqueDescriptorPool global_descriptor_pool;
    vk::UniqueCommandPool command_pool;

    struct SwapchainData
    {
        bool needs_recreation{false};
        vk::UniqueSwapchainKHR swapchain;
        vk::Format format;
        vk::Extent2D extent;
        vk::UniqueRenderPass render_pass;

        struct PerElementData
        {
            vk::Image image;
            vk::UniqueImageView image_view;
            vk::Fence image_fence;
            vk::UniqueFramebuffer framebuffer;
            vk::UniqueCommandBuffer command_buffer;
        };

        std::vector<PerElementData> elements;
    };

    SwapchainData swapchain_data;

    std::unique_ptr<Scene> scene;

    std::size_t current_frame_idx{0};

    struct PerInflightFrameData
    {
        vk::UniqueSemaphore image_available_semaphore;
        vk::UniqueSemaphore rendering_finished_semaphore;
        vk::UniqueFence in_flight_fence;
    };

    std::array<PerInflightFrameData, MAX_FRAMES_IN_FLIGHT> per_inflight_frame_data;
};
