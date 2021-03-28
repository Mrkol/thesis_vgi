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
    std::size_t frequency = 1024;
};

class Resampler
{
public:
    explicit Resampler(const ResamplerConfig& config);

    void Resample(const std::filesystem::path& quad, const std::filesystem::path& info,
        const std::filesystem::path& output_dir, std::size_t thread_idx);

private:
    void BuildPipeline(const ResamplerConfig& config);

private:
    struct PerThreadData
    {
        vk::Queue queue;

        // Order of those fields is important due to destruction order!
        vk::UniqueImage image;
        vk::UniqueImageView image_view;
        vk::UniqueDeviceMemory memory;
        vk::UniqueFramebuffer framebuffer;

        vk::UniqueCommandBuffer command_buffer;

        vk::UniqueFence rendered_fence;
    };

    static constexpr vk::Format OUTPUT_FORMAT = vk::Format::eA8B8G8R8SrgbPack32;

private:
    vk::UniqueInstance vk_instance;
    vk::PhysicalDevice physical_device;
    vk::UniqueDevice device;

    vk::UniqueRenderPass render_pass;
    vk::UniquePipeline graphics_pipeline;
    std::vector<PerThreadData> per_thread_datum;
    vk::UniqueCommandPool command_pool;
};
