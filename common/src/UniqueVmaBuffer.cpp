#include "UniqueVmaBuffer.hpp"


UniqueVmaBuffer::UniqueVmaBuffer(VmaAllocator allocator, std::size_t size,
    vk::BufferUsageFlags buffer_usage, VmaMemoryUsage memory_usage)
    : allocator{allocator}
{
    vk::BufferCreateInfo buf_info{
        {}, size,
        buffer_usage, vk::SharingMode::eExclusive,
        0, nullptr
    };

    VmaAllocationCreateInfo alloc_info{
        .flags = VMA_ALLOCATION_CREATE_MAPPED_BIT,
        .usage = memory_usage
    };

    VkBuffer buf;
    auto retcode = vmaCreateBuffer(allocator, &static_cast<const VkBufferCreateInfo&>(buf_info), &alloc_info,
        &buf, &allocation, nullptr);
    if (retcode != VK_SUCCESS)
    {
        throw std::runtime_error("Unable to create a VAM buffer!");
    }
    buffer = buf;
}

UniqueVmaBuffer::~UniqueVmaBuffer()
{
    if (buffer)
    {
        if (mapped)
        {
            unmap();
        }
        vmaDestroyBuffer(allocator, buffer, allocation);
    }
}

UniqueVmaBuffer::UniqueVmaBuffer(UniqueVmaBuffer&& other) noexcept
{
    allocator = other.allocator;
    allocation = other.allocation;
    buffer = other.buffer;
    mapped = other.mapped;
    other.allocator = {};
    other.allocation = {};
    other.buffer = vk::Buffer{};
    other.mapped = {};
}

UniqueVmaBuffer& UniqueVmaBuffer::operator=(UniqueVmaBuffer&& other) noexcept
{
    if (this == &other)
    {
        return *this;
    }

    if (buffer)
    {
        vmaDestroyBuffer(allocator, buffer, allocation);
    }

    allocator = other.allocator;
    allocation = other.allocation;
    buffer = other.buffer;
    mapped = other.mapped;
    other.allocator = {};
    other.allocation = {};
    other.buffer = vk::Buffer{};
    other.mapped = {};

    return *this;
}

std::byte* UniqueVmaBuffer::map()
{
    void* result;
    if (vmaMapMemory(allocator, allocation, &result) != VK_SUCCESS)
    {
        throw std::runtime_error("Mapping failed");
    }

    return mapped = static_cast<std::byte*>(result);
}

void UniqueVmaBuffer::unmap()
{
    vmaUnmapMemory(allocator, allocation);
    mapped = nullptr;
}
