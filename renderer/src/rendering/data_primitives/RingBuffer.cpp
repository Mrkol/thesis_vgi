#include "RingBuffer.hpp"

#include "Utility.hpp"


static std::size_t align(std::size_t v, std::size_t a)
{
    return (v / a + !!(v % a)) * a;
}

RingBuffer::RingBuffer(const CreateInfo& info)
    : ring_size{info.ring_size}
    , element_size{info.element_size}
    , element_size_with_padding{align(info.element_size, MINIMAL_ALIGNMENT)}
    , buffer(info.allocator, ring_size * element_size_with_padding, info.buffer_usage, info.memory_usage)
{
    buffer.map();
}

RingBuffer::~RingBuffer() noexcept
{
    buffer.unmap();
}

void RingBuffer::write_next(std::span<const std::byte> data)
{
    AD_HOC_ASSERT(data.size() == element_size, "RingBufferResource::write_next called with invalid sized span!");

    memcpy(buffer.data() + current_offset(), data.data(), element_size);
    next();
}

std::span<std::byte> RingBuffer::get_current()
{
    return std::span<std::byte>{buffer.data() + current_offset(), element_size};
}

void RingBuffer::next()
{
    ++current_element;
}

vk::DescriptorBufferInfo RingBuffer::read(std::size_t i)
{
    return vk::DescriptorBufferInfo{
        buffer.get(),
        element_size_with_padding * i,
        element_size
    };
}

std::vector<vk::DescriptorBufferInfo> RingBuffer::read_all()
{
    std::vector<vk::DescriptorBufferInfo> result(ring_size);
    for (std::size_t i = 0; i < ring_size; ++i)
    {
        result[i] = read(i);
    }
    return result;
}