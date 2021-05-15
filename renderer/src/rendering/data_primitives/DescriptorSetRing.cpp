#include "DescriptorSetRing.hpp"

#include "Utility.hpp"


DescriptorSetRing::DescriptorSetRing(const CreateInfo& info)
    : device{info.device}
{
    std::vector layouts(info.ring_size, info.layout);
    descriptor_sets = device.allocateDescriptorSets(vk::DescriptorSetAllocateInfo{
        info.pool, static_cast<uint32_t>(layouts.size()), layouts.data()
    });
}

void DescriptorSetRing::write_all(std::vector<vk::WriteDescriptorSet> writes)
{
    AD_HOC_ASSERT(writes.size() == descriptor_sets.size(), "");

    for (std::size_t i = 0; i < writes.size(); ++i)
    {
        writes[i].dstSet = descriptor_sets[i];
    }
    device.updateDescriptorSets(writes, {});
}

void DescriptorSetRing::write_ubo(RingBuffer& rb, uint32_t binding)
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

void DescriptorSetRing::write_sbo(RingBuffer& rb, uint32_t binding)
{
    std::vector<vk::DescriptorBufferInfo> buffer_infos = rb.read_all();
    std::vector writes(descriptor_sets.size(), vk::WriteDescriptorSet{
        {}, binding, 0, 1, vk::DescriptorType::eStorageBuffer, nullptr, nullptr, nullptr
    });

    for (std::size_t i = 0; i < writes.size(); ++i)
    {
        writes[i].pBufferInfo = &buffer_infos[i];
    }

    write_all(writes);
}

vk::DescriptorSet DescriptorSetRing::read_next()
{
    auto ours = (current++) % descriptor_sets.size();
    return descriptor_sets[ours];
}