#pragma once

#include <vulkan/vulkan.hpp>
#include <vk_mem_alloc.h>


class UniqueVmaImage
{
public:
    UniqueVmaImage() = default;

    UniqueVmaImage(VmaAllocator allocator, vk::Format format, vk::Extent2D extent,
        vk::ImageTiling tiling, vk::ImageUsageFlags image_usage, VmaMemoryUsage memory_usage, std::size_t layers = 1);

    UniqueVmaImage(const UniqueVmaImage&) = delete;
    UniqueVmaImage& operator=(const UniqueVmaImage&) = delete;

    void swap(UniqueVmaImage& other);
    UniqueVmaImage(UniqueVmaImage&&) noexcept;
    UniqueVmaImage& operator=(UniqueVmaImage&&) noexcept;

    [[nodiscard]] vk::Image get() const { return image; }

    void transfer_layout(vk::CommandBuffer cb,
        vk::ImageLayout src, vk::ImageLayout dst,
        vk::AccessFlags srcAccess, vk::AccessFlags dstAccess,
        vk::PipelineStageFlags srcStages, vk::PipelineStageFlags dstStages);

    ~UniqueVmaImage();

private:
    VmaAllocator allocator{};
    std::size_t layers{};

    VmaAllocation allocation{};
    vk::Image image{};
};
