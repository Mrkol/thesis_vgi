#pragma once

#include <vulkan/vulkan.hpp>
#include <vk_mem_alloc.h>


class UniqueVmaImage
{
public:
    UniqueVmaImage() = default;

    UniqueVmaImage(VmaAllocator allocator, std::size_t size,
        vk::BufferUsageFlags buffer_usage, VmaMemoryUsage memory_usage);

    UniqueVmaImage(const UniqueVmaImage&) = delete;
    UniqueVmaImage& operator=(const UniqueVmaImage&) = delete;

    UniqueVmaImage(UniqueVmaImage&&) noexcept;
    UniqueVmaImage& operator=(UniqueVmaImage&&) noexcept;

    [[nodiscard]] vk::UniqueImageView view() const;

    ~UniqueVmaImage();

private:
    VmaAllocator allocator{};

    VmaAllocation allocation{};
    vk::UniqueImage buffer{};
};
