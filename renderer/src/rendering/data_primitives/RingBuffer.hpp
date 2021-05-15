#pragma once

#include <vulkan/vulkan.hpp>
#include <vk_mem_alloc.h>
#include <span>
#include "UniqueVmaBuffer.hpp"


// TODO: make this configurable as different operations need different alignment
constexpr std::size_t MINIMAL_ALIGNMENT = 256*3;

/**
 * Synchronized by inflight fences in Renderer
 */
class RingBuffer
{
public:
    RingBuffer() = default;

    struct CreateInfo
    {
        VmaAllocator allocator;
        std::size_t ring_size;
        std::size_t element_size;
        vk::BufferUsageFlags buffer_usage;
        VmaMemoryUsage memory_usage;
    };

    explicit RingBuffer(const CreateInfo& info);

    RingBuffer(RingBuffer&&) = default;
    RingBuffer& operator=(RingBuffer&&) = default;

    ~RingBuffer() noexcept;

    void write_next(std::span<const std::byte> data);
    std::span<std::byte> get_current();
    void next();

    [[nodiscard]] vk::DescriptorBufferInfo read(std::size_t i);
    [[nodiscard]] std::vector<vk::DescriptorBufferInfo> read_all();

    [[nodiscard]] vk::Buffer get() const { return buffer.get(); }
    [[nodiscard]] std::size_t current_offset() const { return element_size_with_padding * current_index(); }
    [[nodiscard]] std::size_t current_index() const { return current_element % ring_size; }

    [[nodiscard]] std::size_t size() const { return ring_size; }

private:
    std::size_t ring_size{};
    std::size_t element_size{};
    std::size_t element_size_with_padding{};

    std::size_t current_element{};

    UniqueVmaBuffer buffer;
};
