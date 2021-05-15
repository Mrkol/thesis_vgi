#pragma once

#include "UniqueVmaBuffer.hpp"
#include "data_primitives/VirtualTextureSet.hpp"
#include "data_primitives/RingBuffer.hpp"
#include "data_primitives/DescriptorSetRing.hpp"


class IResourceManager
{
public:
    virtual vk::Device get_device() = 0;

    virtual RingBuffer create_ubo(std::size_t size) = 0;
    virtual RingBuffer create_sbo(std::size_t size) = 0;
    virtual DescriptorSetRing create_descriptor_set_ring(vk::DescriptorSetLayout layout) = 0;
    virtual vk::UniqueDescriptorSet create_descriptor_set(vk::DescriptorSetLayout layout) = 0;
    virtual UniqueVmaBuffer create_vbo(std::size_t size) = 0;

    virtual VirtualTextureSet create_svt(
        std::size_t gpu_cache_side_size,
        std::size_t per_frame_update_limit,
        vk::Format format,
        std::size_t min_mip,
        std::vector<std::vector<const std::byte*>> image_mip_data
    ) = 0;

    virtual vk::UniqueCommandBuffer begin_single_time_commands() = 0;
    virtual void finish_single_time_commands(vk::UniqueCommandBuffer cb) = 0;

    virtual  ~IResourceManager() = default;
};
