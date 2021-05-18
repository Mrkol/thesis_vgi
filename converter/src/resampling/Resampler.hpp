#pragma once

#include <filesystem>
#include <vector>
#include <vulkan/vulkan.hpp>

#include <UniqueVmaBuffer.hpp>
#include <UniqueVmaImage.hpp>

#include "../clustering/Quadrangulation.hpp"
#include "../Parametrization.hpp"


#ifdef NDEBUG
constexpr std::array<const char*, 0> VALIDATION_LAYERS {};
#else
constexpr std::array VALIDATION_LAYERS {"VK_LAYER_KHRONOS_validation"};
#endif

struct ResamplerConfig
{
    std::size_t thread_count{0};
    std::size_t log_resolution = 7;
};

class Resampler
{
public:
    explicit Resampler(const ResamplerConfig& config);

    std::vector<float> resample(const QuadPatch& patch,
        const std::vector<MappingElement>& mapping, std::size_t thread_idx);

private:
    void build_pipeline();
    void build_command_buffer(vk::Buffer vertex_buffer, vk::Buffer triangle_buffer, vk::Buffer line_buffer,
        uint32_t triangle_indices_count, uint32_t line_indices_count, std::size_t thread_idx);

private:
    static constexpr std::size_t ATTACHMENT_COUNT = 2;

    struct PerThreadData
    {
        vk::Queue queue;

        std::array<UniqueVmaBuffer, ATTACHMENT_COUNT> image_staging;

        // Order of those fields is important due to destruction order!
        std::array<UniqueVmaImage, ATTACHMENT_COUNT> images;
        std::array<vk::UniqueImageView, ATTACHMENT_COUNT> image_views;
        vk::UniqueFramebuffer framebuffer;

        vk::UniqueCommandBuffer command_buffer;
    };

    static constexpr vk::Format OUTPUT_FORMAT = vk::Format::eR32G32B32A32Sfloat;
    static constexpr std::size_t OUTPUT_PIXEL_SIZE = sizeof(float) * 4;

private:
    vk::UniqueInstance vk_instance;
    vk::PhysicalDevice physical_device;
    vk::UniqueDevice device;
    VmaAllocator allocator;
    std::unique_ptr<void, void(*)(void*)> deferred_allocator_destroy{nullptr, nullptr};

    uint32_t resampling_resolution;

    UniqueVmaBuffer uniform_buffer;

    vk::UniqueDescriptorPool descriptor_pool;
    vk::DescriptorSet descriptor_set;

    vk::UniqueRenderPass render_pass;
    vk::UniquePipelineLayout pipeline_layout;
    vk::UniquePipeline triangle_pipeline;
    vk::UniquePipeline line_pipeline;

    vk::UniqueCommandPool command_pool;
    std::vector<PerThreadData> per_thread_datum;
};
