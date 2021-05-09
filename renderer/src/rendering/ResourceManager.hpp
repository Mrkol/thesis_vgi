#pragma once

#include <vulkan/vulkan.hpp>
#include <vk_mem_alloc.h>
#include <span>

#include "UniqueVmaBuffer.hpp"


// all GPUs support this
constexpr std::size_t MINIMAL_ALIGNMENT = 256;

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

    void write_next(std::span<std::byte> data);
    [[nodiscard]] vk::DescriptorBufferInfo read(std::size_t i);
    [[nodiscard]] std::vector<vk::DescriptorBufferInfo> read_all();

    [[nodiscard]] std::size_t size() const { return ring_size; }

private:
    std::size_t ring_size{};
    std::size_t element_size{};
    std::size_t element_size_with_padding{};

    std::size_t current_element{};

    UniqueVmaBuffer buffer;
};

class UniformRing
{
public:
    UniformRing() = default;

    struct CreateInfo
    {
        vk::Device device;
        vk::DescriptorPool pool;
        std::size_t ring_size;
        vk::DescriptorSetLayout layout;
    };

    explicit UniformRing(const CreateInfo& info);

    void write_all(std::vector<vk::WriteDescriptorSet> writes);
    void write_ubo(RingBuffer& rb, uint32_t binding);
    vk::DescriptorSet read_next();



    [[nodiscard]] std::size_t size() const { return descriptor_sets.size(); }

private:
    vk::Device device;
    std::vector<vk::DescriptorSet> descriptor_sets;
    std::size_t current{};
};

class IResourceManager
{
public:
    virtual vk::Device get_device() = 0;
    virtual RingBuffer create_ubo(std::size_t size) = 0;
    virtual UniformRing create_descriptor_set(vk::DescriptorSetLayout layout) = 0;
    virtual UniqueVmaBuffer create_vbo(std::size_t size) = 0;

    virtual vk::UniqueCommandBuffer begin_single_time_commands() = 0;
    virtual void finish_single_time_commands(vk::UniqueCommandBuffer cb) = 0;

    virtual  ~IResourceManager() = default;
};
