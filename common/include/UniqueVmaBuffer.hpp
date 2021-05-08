#pragma once

#include <vulkan/vulkan.hpp>
#include <vk_mem_alloc.h>


class UniqueVmaBuffer
{
public:
    UniqueVmaBuffer() = default;

    UniqueVmaBuffer(VmaAllocator allocator, std::size_t size,
        vk::BufferUsageFlags buffer_usage, VmaMemoryUsage memory_usage);

    UniqueVmaBuffer(const UniqueVmaBuffer&) = delete;
    UniqueVmaBuffer& operator=(const UniqueVmaBuffer&) = delete;

    UniqueVmaBuffer(UniqueVmaBuffer&&) noexcept;
    UniqueVmaBuffer& operator=(UniqueVmaBuffer&&) noexcept;

    [[nodiscard]] vk::Buffer get() const { return buffer; }
    [[nodiscard]] std::byte* data() { return mapped; }

    std::byte* map();
    void unmap();

    ~UniqueVmaBuffer();

private:
    VmaAllocator allocator{};

    VmaAllocation allocation{};
    vk::Buffer buffer{};
    std::byte* mapped{};
};
