#pragma once

#include <unordered_set>
#include <Eigen/Dense>

#include "UniqueVmaImage.hpp"
#include "RingBuffer.hpp"


class VirtualTextureSet
{
public:
    VirtualTextureSet() = default;

    struct CreateInfo
    {
        vk::Device device;
        VmaAllocator allocator;
        vk::CommandBuffer single_time_cb;

        std::size_t multiple_buffer_size;

        // Actual cache is gpu_cache_side_size * gpu_cache_side_size pages
        std::size_t gpu_cache_side_size;

        // Maximum amount of pages that can be updated per frame
        std::size_t per_frame_update_limit;

        vk::Format format;
        std::size_t format_multiplicity;

        // Minimum mipmap level. Also determines the cache page side size as 2^min_mip
        std::size_t min_mip;

        // image_mip_data[i][j] is the ith image with jth mip level (therefore the size is 2^(min_mip + j)
        std::vector<std::vector<const std::byte*>> image_mip_data;
    };

    explicit VirtualTextureSet(CreateInfo);

    constexpr static auto NO_PAGE = std::numeric_limits<std::size_t>::max();

    struct PageInfo
    {
        std::size_t image_index{NO_PAGE};
        std::size_t image_mip;
        std::size_t x;
        std::size_t y;
    };

    void bump_page(PageInfo info);

    void bump_region(std::size_t index, std::size_t mip, float x, float y, float size);

    void tick();

    void record_commands(vk::CommandBuffer cb);

    vk::ImageView view() const { return cache_view.get(); }
    vk::Sampler sampler() const { return cache_sampler.get(); }
    RingBuffer& get_indirection_table_sbo() { return indirection_tables_sbo; }

    [[nodiscard]] std::size_t get_mip_level_count() const { return image_mip_data[0].size(); }
    [[nodiscard]] std::size_t cache_size_pixels() const
        { return cache_side_size * cache_side_size * page_side_size * page_side_size; }

private:
    void bump_page_impl(PageInfo info, std::size_t gen);

    [[nodiscard]] std::size_t page_size() const { return pixel_size * page_side_size * page_side_size; }

    void transfer_layout(vk::CommandBuffer cb,
        vk::ImageLayout src, vk::ImageLayout dst,
        vk::AccessFlags srcAccess, vk::AccessFlags dstAccess,
        vk::PipelineStageFlags srcStages, vk::PipelineStageFlags dstStages);

private:
    std::size_t min_mip{0};
    std::size_t pixel_size{0};
    std::size_t page_side_size{0};
    std::size_t per_frame_update_limit{0};
    std::size_t format_multiplicity{0};

    std::vector<std::vector<const std::byte*>> image_mip_data;

    // When an upload is requested, pages get staged here and uploaded via a copy command later on (multiple buffered)
    RingBuffer staging_buffer;
    // Coords in cache where we want the staged pages to land
    std::vector<vk::BufferImageCopy> staging_targets;

    class IndirectionTables
    {
    public:
        static constexpr uint32_t EMPTY = std::numeric_limits<uint32_t>::max();

        IndirectionTables() = default;

        IndirectionTables(std::size_t level_count, std::size_t table_count);

        [[nodiscard]] uint32_t& operator() (std::size_t idx, std::size_t size, std::size_t x, std::size_t y);

        [[nodiscard]] const std::byte* get_data() const { return reinterpret_cast<const std::byte*>(data.data()); }
        [[nodiscard]] std::size_t size() const { return data.size() * sizeof(uint32_t); }

    private:
        std::size_t level_count{0};
        std::vector<uint32_t> data;
    };


    // Lookup tables to get the appropriate page from cache (multiple buffered)
    IndirectionTables indirection_tables;
    RingBuffer indirection_tables_sbo;

    // Only 1 render pass is running at a time therefore we don't need to multiple buffer this
    std::size_t cache_side_size{0};
    UniqueVmaImage cache;
    std::vector<std::size_t> cache_lifetimes;
    std::vector<PageInfo> cache_state;
    std::size_t generation{1};

    vk::UniqueImageView cache_view;
    vk::UniqueSampler cache_sampler;
};
