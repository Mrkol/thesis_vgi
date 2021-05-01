#pragma once

#include <vulkan/vulkan.hpp>
#include <function2/function2.hpp>
#include <span>


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

private:
    fu2::unique_function<vk::Extent2D()> resolution_provider;

    vk::Instance* vulkan_instance;
    vk::UniqueSurfaceKHR display_surface;
    vk::PhysicalDevice physical_device;
    vk::UniqueDevice device;
    vk::Queue graphics_queue;
};
