#pragma once

#include <vulkan/vulkan.hpp>
#include <function2/function2.hpp>
#include <span>


struct QueueFamilyIndices
{
    uint32_t graphics_queue_idx;
    uint32_t presentation_queue_idx;
};

class Renderer {
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

private:
    vk::Extent2D chose_swap_extent(const vk::SurfaceCapabilitiesKHR& capabilities);

    // TODO: this procedural-like initialization pattern is questionable
    void create_swapchain();

private:
    fu2::unique_function<vk::Extent2D()> resolution_provider;

    vk::Instance* vulkan_instance;
    vk::UniqueSurfaceKHR display_surface;
    vk::PhysicalDevice physical_device;

    QueueFamilyIndices queue_family_indices;
    vk::UniqueDevice device;
    vk::Queue graphics_queue;
    vk::Queue present_queue;

    vk::UniqueSwapchainKHR swapchain;
    std::vector<vk::Image> swapchain_images;
    std::vector<vk::UniqueImageView> swapchain_image_views;
    vk::Format swapchain_format;
    vk::Extent2D swapchain_extent;
};
