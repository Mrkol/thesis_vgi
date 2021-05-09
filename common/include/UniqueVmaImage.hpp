#pragma once

#include <vulkan/vulkan.hpp>
#include <vk_mem_alloc.h>


class UniqueVmaImage
{
public:
    UniqueVmaImage() = default;

    UniqueVmaImage(VmaAllocator allocator, vk::Format format, vk::Extent2D extent,
        vk::ImageTiling tiling, vk::ImageUsageFlags image_usage, VmaMemoryUsage memory_usage);

    UniqueVmaImage(const UniqueVmaImage&) = delete;
    UniqueVmaImage& operator=(const UniqueVmaImage&) = delete;

    UniqueVmaImage(UniqueVmaImage&&) noexcept;
    UniqueVmaImage& operator=(UniqueVmaImage&&) noexcept;

    vk::Image get() const { return image; }

    ~UniqueVmaImage();

private:
    VmaAllocator allocator{};

    VmaAllocation allocation{};
    vk::Image image{};
};
