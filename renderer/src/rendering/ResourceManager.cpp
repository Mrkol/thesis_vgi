#include "ResourceManager.hpp"


#include "Utility.hpp"


static std::size_t align(std::size_t v, std::size_t a)
{
    return (v / a + !!(v % a)) * a;
}

RingBuffer::RingBuffer(const CreateInfo& info)
    : ring_size{info.ring_size}
    , element_size{info.element_size}
    , element_size_with_padding{align(info.element_size, MINIMAL_ALIGNMENT)}
    , buffer(info.allocator, info.ring_size * element_size_with_padding, info.buffer_usage, info.memory_usage)
{
    buffer.map();
}

void RingBuffer::write_next(std::span<std::byte> data)
{
    AD_HOC_ASSERT(data.size() == element_size, "RingBufferResource::write_next called with invalid sized span!");

    auto ours = (current_element++) % ring_size;

    std::memcpy(buffer.data() + ours * element_size_with_padding, data.data(), element_size);
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

UniformRing::UniformRing(const CreateInfo& info)
    : device{info.device}
{
    std::vector layouts(info.ring_size, info.layout);
    descriptor_sets = device.allocateDescriptorSets(vk::DescriptorSetAllocateInfo{
        info.pool, static_cast<uint32_t>(layouts.size()), layouts.data()
    });
}

void UniformRing::write_all(std::vector<vk::WriteDescriptorSet> writes)
{
    AD_HOC_ASSERT(writes.size() == descriptor_sets.size(), "");

    for (std::size_t i = 0; i < writes.size(); ++i)
    {
        writes[i].dstSet = descriptor_sets[i];
    }
    device.updateDescriptorSets(writes, {});
}

void UniformRing::write_ubo(RingBuffer& rb, uint32_t binding)
{
    std::vector<vk::DescriptorBufferInfo> buffer_infos = rb.read_all();
    std::vector writes(descriptor_sets.size(), vk::WriteDescriptorSet{
        {}, binding, 0, 1, vk::DescriptorType::eUniformBuffer, nullptr, nullptr, nullptr
    });

    for (std::size_t i = 0; i < writes.size(); ++i)
    {
        writes[i].pBufferInfo = &buffer_infos[i];
    }

    write_all(writes);
}

vk::DescriptorSet UniformRing::read_next()
{
    auto ours = (current++) % descriptor_sets.size();
    return descriptor_sets[ours];
}
