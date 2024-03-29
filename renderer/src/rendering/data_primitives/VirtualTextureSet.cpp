#include "VirtualTextureSet.hpp"

#include <Utility.hpp>


static std::unordered_map<vk::Format, std::size_t> FORMAT_SIZE{
    {vk::Format::eR32G32B32Sfloat, 4*3},
    {vk::Format::eR32G32B32A32Uint, 4*4},
    {vk::Format::eR32G32B32A32Sfloat, 4*4}
};

std::size_t mip_to_size(std::size_t mip)
{
    return std::size_t{1} + (std::size_t{1} << mip);
}

VirtualTextureSet::VirtualTextureSet(VirtualTextureSet::CreateInfo info)
    : min_mip{info.min_mip}
    , pixel_size(FORMAT_SIZE.at(info.format))
    , page_side_size(mip_to_size(info.min_mip))
    , image_mip_data(std::move(info.image_mip_data))
    , cache_side_size(info.gpu_cache_side_size)
    , per_frame_update_limit(info.per_frame_update_limit)
    , format_multiplicity(info.format_multiplicity)
{
    staging_buffer = RingBuffer({info.allocator, info.multiple_buffer_size,
        format_multiplicity * per_frame_update_limit * page_size(),
        vk::BufferUsageFlagBits::eTransferSrc, VmaMemoryUsage::VMA_MEMORY_USAGE_CPU_TO_GPU});

    indirection_tables = IndirectionTables(
        image_mip_data[0].size(), image_mip_data.size());

    indirection_tables_sbo = RingBuffer({info.allocator, info.multiple_buffer_size,
        indirection_tables.size(),
        vk::BufferUsageFlagBits::eStorageBuffer, VmaMemoryUsage::VMA_MEMORY_USAGE_CPU_TO_GPU});



    cache = UniqueVmaImage(info.allocator, info.format,
        vk::Extent2D{
            static_cast<uint32_t>(info.gpu_cache_side_size * page_side_size),
            static_cast<uint32_t>(info.gpu_cache_side_size * page_side_size)},
        vk::ImageTiling::eOptimal, vk::ImageUsageFlagBits::eTransferDst | vk::ImageUsageFlagBits::eSampled,
        VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY, info.format_multiplicity);

    cache_lifetimes.resize(cache_side_size*cache_side_size, 0);
    cache_state.resize(cache_lifetimes.size());

    // Trick: we chain this barrier with the one in record_commands
    cache.transfer_layout(info.single_time_cb,
        vk::ImageLayout::eUndefined, vk::ImageLayout::eShaderReadOnlyOptimal,
        {}, {},
        vk::PipelineStageFlagBits::eTopOfPipe, vk::PipelineStageFlagBits::eTessellationEvaluationShader);

    for (std::size_t i = 0; i < image_mip_data.size(); ++i)
    {
        bump_page_impl(PageInfo{i, min_mip, 0, 0}, std::numeric_limits<std::size_t>::max());
    }

    cache_view = info.device.createImageViewUnique(vk::ImageViewCreateInfo{
        {}, cache.get(), vk::ImageViewType::e2DArray, info.format, {},
        vk::ImageSubresourceRange{
            vk::ImageAspectFlagBits::eColor, 0, 1, 0, static_cast<uint32_t>(format_multiplicity)
        }
    });

    cache_sampler = info.device.createSamplerUnique(vk::SamplerCreateInfo{
        {},
        vk::Filter::eLinear, vk::Filter::eNearest,
        vk::SamplerMipmapMode::eLinear,
        /* u */ vk::SamplerAddressMode::eClampToBorder,
        /* v */ vk::SamplerAddressMode::eClampToBorder,
        /* w */ vk::SamplerAddressMode::eClampToBorder,
        0,
        /* anisotropy */ false, {},
        /* compare */ false, {},
        0, 0,
        vk::BorderColor::eIntOpaqueBlack, false
    });
}

std::tuple<std::size_t, std::size_t, std::size_t, std::size_t> to_tuple(VirtualTextureSet::PageInfo info)
{
    return {info.image_index, info.image_mip, info.x, info.y};
}

void VirtualTextureSet::bump_page(VirtualTextureSet::PageInfo info)
{
    bump_page_impl(info, generation);
}

void VirtualTextureSet::bump_page_impl(VirtualTextureSet::PageInfo info, std::size_t gen)
{
    if (access_indirection_table(info) != IndirectionTables::EMPTY)
    {
        return;
    }

    if (staging_targets.size() >= per_frame_update_limit * format_multiplicity)
    {
        return;
    }

    std::size_t lru = std::min_element(cache_lifetimes.begin(), cache_lifetimes.end()) - cache_lifetimes.begin();

    if (auto prev_state = cache_state[lru]; prev_state.image_index != NO_PAGE)
    {
        access_indirection_table(prev_state) = IndirectionTables::EMPTY;
    }
    cache_state[lru] = info;

    access_indirection_table(info) = uint32_t(lru);
    cache_lifetimes[lru] = gen;

    for (std::size_t i = 0; i < format_multiplicity; ++i)
    {
        std::size_t staging_offset = staging_targets.size() * page_size();
        staging_targets.emplace_back(vk::BufferImageCopy{
            staging_buffer.current_offset() + staging_offset,
            static_cast<uint32_t>(page_side_size), static_cast<uint32_t>(page_side_size),
            vk::ImageSubresourceLayers{
                vk::ImageAspectFlagBits::eColor,
                0, static_cast<uint32_t>(i), 1
            },
            vk::Offset3D{
                static_cast<int32_t>(lru % cache_side_size * page_side_size),
                static_cast<int32_t>(lru / cache_side_size * page_side_size),
                0
            },
            vk::Extent3D{
                static_cast<uint32_t>(page_side_size), static_cast<uint32_t>(page_side_size), 1
            }
        });

        // -1 here is for the overlap
        std::byte* dst = staging_buffer.get_current().data() + staging_offset;

        std::size_t gi_offset =
            info.y * (page_side_size - 1) * (pixel_size * format_multiplicity) * mip_to_size(info.image_mip)
                + info.x * (page_side_size - 1) * (pixel_size * format_multiplicity);

        const std::byte* src = image_mip_data.at(info.image_index).at(info.image_mip - min_mip) + gi_offset;

        for (std::size_t y = 0; y < page_side_size; ++y)
        {
            auto row_src = src;
            for (std::size_t x = 0; x < page_side_size; ++x)
            {
                std::memcpy(dst, row_src + i*pixel_size, pixel_size);
                row_src += format_multiplicity * pixel_size;
                dst += pixel_size;
            }
            src += format_multiplicity * pixel_size * mip_to_size(info.image_mip);
        }
    }
}

uint32_t& VirtualTextureSet::access_indirection_table(VirtualTextureSet::PageInfo info)
{
    return indirection_tables(info.image_index, info.image_mip - min_mip, info.x, info.y);
}

std::size_t VirtualTextureSet::bump_region(std::size_t index, std::size_t mip, float x, float y, float size)
{
    std::size_t total_size = mip_to_size(mip);

    auto px_size = static_cast<std::size_t>(std::ceil(size * float(total_size - 1) / float(page_side_size - 1)));

    auto px_x = static_cast<std::size_t>(x * float(total_size - 1) / float(page_side_size - 1));
    auto px_y = static_cast<std::size_t>(y * float(total_size - 1) / float(page_side_size - 1));

    std::size_t result = mip;
    for (std::size_t i = 0; i < px_size; ++i)
    {
        for (std::size_t j = 0; j < px_size; ++j)
        {
            PageInfo page{
                index,
                mip,
                px_x + i,
                px_y + j
            };

            bump_page(page);

            // Only account for side mips
            if (i == 0 || j == 0 || i == px_size - 1 || j == px_size - 1)
            {
                while (page.image_mip > 0 && access_indirection_table(page) == IndirectionTables::EMPTY)
                {
                    --page.image_mip;
                    page.x /= 2;
                    page.y /= 2;
                }

                result = std::min(result, page.image_mip);
            }
        }
    }

    return result;
}

void VirtualTextureSet::tick()
{
    indirection_tables_sbo.write_next(std::span{indirection_tables.get_data(), indirection_tables.size()});
    staging_buffer.next();
    ++generation;
}

void VirtualTextureSet::record_commands(vk::CommandBuffer cb)
{
    if (!staging_targets.empty())
    {
        cache.transfer_layout(cb,
            vk::ImageLayout::eShaderReadOnlyOptimal, vk::ImageLayout::eTransferDstOptimal,
            {}, vk::AccessFlagBits::eTransferWrite,
            vk::PipelineStageFlagBits::eTessellationEvaluationShader, vk::PipelineStageFlagBits::eTransfer);

        cb.copyBufferToImage(staging_buffer.get(), cache.get(), vk::ImageLayout::eTransferDstOptimal,
            static_cast<uint32_t>(staging_targets.size()), staging_targets.data());
        staging_targets.clear();

        cache.transfer_layout(cb,
            vk::ImageLayout::eTransferDstOptimal, vk::ImageLayout::eShaderReadOnlyOptimal,
            vk::AccessFlagBits::eTransferWrite, vk::AccessFlagBits::eShaderRead,
            vk::PipelineStageFlagBits::eTransfer, vk::PipelineStageFlagBits::eTessellationEvaluationShader);
    }
}

std::size_t calc_indirection_tables_size(std::size_t count)
{
    return ((std::size_t{1} << (2*count)) - 1)/3;
}

VirtualTextureSet::IndirectionTables::IndirectionTables(std::size_t level_count, std::size_t table_count)
    : level_count{level_count}
    , data(table_count * calc_indirection_tables_size(level_count), EMPTY)
{

}

uint32_t& VirtualTextureSet::IndirectionTables::operator ()(
    std::size_t idx, std::size_t size, std::size_t x, std::size_t y)
{
    AD_HOC_ASSERT(size < level_count, "Incorrect size");
    AD_HOC_ASSERT(x < (std::size_t{1} << size) && y < (std::size_t{1} << size), "Incorrect position");
    std::size_t i =
        idx * calc_indirection_tables_size(level_count)
        + calc_indirection_tables_size(size)
        + y * (std::size_t{1} << size)
        + x;

    AD_HOC_ASSERT(i < data.size(), "Incorrect index");

    return data[i];
}
