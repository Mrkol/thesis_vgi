#include "Texture.hpp"

#include "../ResourceManager.hpp"
#include "../../UniqueStbImage.hpp"
#include "Utility.hpp"


Texture::Texture(std::span<std::filesystem::path> layers, IResourceManager* irm)
{
    AD_HOC_ASSERT(!layers.empty(), "Empty textures not supported");

    std::vector<UniqueStbImage> raw;
    raw.reserve(layers.size());
    for (auto& path : layers)
    {
        raw.emplace_back(path);
    }

    for (auto& r : raw)
    {
        AD_HOC_ASSERT(r.width() == raw[0].width() && r.height() == raw[0].height(),
            "All layers must have the same format");
    }


    textures_ = irm->create_texture(
        {static_cast<uint32_t>(raw[0].width()), static_cast<uint32_t>(raw[0].height())}, raw.size());


    auto cb = irm->begin_single_time_commands();

    textures_.transfer_layout(cb.get(), vk::ImageLayout::eUndefined, vk::ImageLayout::eTransferDstOptimal,
        {}, vk::AccessFlagBits::eTransferWrite,
        vk::PipelineStageFlagBits::eTopOfPipe, vk::PipelineStageFlagBits::eTransfer);


    std::size_t staging_size = raw.size() * raw[0].width() * raw[0].height() * 4;
    auto staging = irm->create_staging_buffer(staging_size);

    auto mapped = staging.map();

    std::vector<std::size_t> offsets;
    offsets.reserve(raw.size());

    std::size_t curr_offset = 0;
    for (auto& map : raw)
    {
        offsets.push_back(curr_offset);
        std::size_t size = map.width() * map.height() * 4;
        std::memcpy(mapped + curr_offset, map.data(), size);
        curr_offset += size;
    }

    staging.unmap();

    std::vector<vk::BufferImageCopy> copy_ops;
    copy_ops.reserve(raw.size());
    for (auto& r : raw)
    {
        copy_ops.push_back(vk::BufferImageCopy{
            static_cast<uint32_t>(offsets[copy_ops.size()]),
            static_cast<uint32_t>(r.width()), static_cast<uint32_t>(r.height()),
            vk::ImageSubresourceLayers{
                vk::ImageAspectFlagBits::eColor,
                0, static_cast<uint32_t>(copy_ops.size()), 1
            },
            vk::Offset3D{0, 0, 0},
            vk::Extent3D{
                static_cast<uint32_t>(r.width()),
                static_cast<uint32_t>(r.height()),
                1
            }
        });
    }

    cb->copyBufferToImage(staging.get(), textures_.get(), vk::ImageLayout::eTransferDstOptimal, copy_ops);

    textures_.transfer_layout(cb.get(), vk::ImageLayout::eTransferDstOptimal, vk::ImageLayout::eShaderReadOnlyOptimal,
        vk::AccessFlagBits::eTransferWrite, vk::AccessFlagBits::eShaderRead,
        vk::PipelineStageFlagBits::eTransfer, vk::PipelineStageFlagBits::eFragmentShader);

    irm->finish_single_time_commands(std::move(cb));



    textures_view_ = irm->get_device().createImageViewUnique(vk::ImageViewCreateInfo{
        {}, textures_.get(), vk::ImageViewType::e2DArray, vk::Format::eR8G8B8A8Srgb,
        vk::ComponentMapping{},
        vk::ImageSubresourceRange{
            vk::ImageAspectFlagBits::eColor,
            0, 1, 0, static_cast<uint32_t>(raw.size())
        }
    });
}
