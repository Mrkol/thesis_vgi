#pragma once

#include <vulkan/vulkan.hpp>
#include <vk_mem_alloc.h>
#include <span>
#include "RingBuffer.hpp"


class DescriptorSetRing
{
public:
    DescriptorSetRing() = default;

    struct CreateInfo
    {
        vk::Device device;
        vk::DescriptorPool pool;
        std::size_t ring_size{1};
        vk::DescriptorSetLayout layout;
    };

    explicit DescriptorSetRing(const CreateInfo& info);

    void write_all(std::vector<vk::WriteDescriptorSet> writes);
    void write_ubo(RingBuffer& rb, uint32_t binding);
    void write_sbo(RingBuffer& rb, uint32_t binding);
    vk::DescriptorSet read_next();

    [[nodiscard]] std::size_t size() const { return descriptor_sets.size(); }

private:
    vk::Device device;
    std::vector<vk::DescriptorSet> descriptor_sets;
    std::size_t current{};
};
