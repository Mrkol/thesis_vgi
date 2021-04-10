#pragma once

#include <filesystem>
#include <vector>
#include <vulkan/vulkan.hpp>


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

    void resample(const std::filesystem::path& quad, const std::filesystem::path& info_dir,
        const std::filesystem::path& output_dir, std::size_t thread_idx);

private:
    void build_pipeline(const ResamplerConfig& config);
    void build_command_buffer(vk::Buffer vertex_buffer, vk::Buffer triangle_buffer, vk::Buffer line_buffer,
        uint32_t triangle_indices_count, uint32_t line_indices_count, std::size_t thread_idx);

private:
    struct PerThreadData
    {
        vk::Queue queue;

        // Order of those fields is important due to destruction order!
        vk::UniqueDeviceMemory image_memory;
        vk::UniqueImage image;
        vk::UniqueImageView image_view;
        vk::UniqueFramebuffer framebuffer;

        vk::UniqueCommandBuffer command_buffer;

        vk::UniqueFence rendered_fence;
    };

    static constexpr vk::Format OUTPUT_FORMAT = vk::Format::eR32G32B32A32Sfloat;

private:
    vk::UniqueInstance vk_instance;
    vk::PhysicalDevice physical_device;
    vk::UniqueDevice device;

    uint32_t resampling_resolution;

    vk::UniqueDeviceMemory  uniform_buffer_memory;
    vk::UniqueBuffer uniform_buffer;

    vk::UniqueDescriptorPool descriptor_pool;
    vk::DescriptorSet descriptor_set;

    vk::UniqueRenderPass render_pass;
    vk::UniquePipelineLayout pipeline_layout;
    vk::UniquePipeline triangle_pipeline;
    vk::UniquePipeline line_pipeline;

    vk::UniqueCommandPool command_pool;
    std::vector<PerThreadData> per_thread_datum;
};
