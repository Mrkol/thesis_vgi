#include "UniqueVmaImage.hpp"


UniqueVmaImage::UniqueVmaImage(VmaAllocator allocator, vk::Format format, vk::Extent2D extent,
    vk::ImageTiling tiling, vk::ImageUsageFlags image_usage, VmaMemoryUsage memory_usage, std::size_t layers)
    : allocator{allocator}
    , layers{layers}
{
    vk::ImageCreateInfo image_info{
        {}, vk::ImageType::e2D, format, vk::Extent3D{extent.width, extent.height, 1},
        1,
        static_cast<uint32_t>(layers),
        vk::SampleCountFlagBits::e1, tiling, image_usage, vk::SharingMode::eExclusive,
        0, nullptr, vk::ImageLayout::eUndefined
    };
    VmaAllocationCreateInfo alloc_info{
        .usage = memory_usage
    };
    VkImage img;

    auto retcode = vmaCreateImage(allocator, &static_cast<const VkImageCreateInfo&>(image_info), &alloc_info,
            &img, &allocation, nullptr);
    if (retcode != VK_SUCCESS)
    {
        throw std::runtime_error("Unable to create a VMA image!");
    }
    image = img;
}

void UniqueVmaImage::swap(UniqueVmaImage& other)
{
    std::swap(allocator, other.allocator);
    std::swap(allocation, other.allocation);
    std::swap(image, other.image);
    std::swap(layers, other.layers);
}

UniqueVmaImage::UniqueVmaImage(UniqueVmaImage&& other) noexcept
{
    swap(other);
}

UniqueVmaImage& UniqueVmaImage::operator =(UniqueVmaImage&& other) noexcept
{
    if (this == &other)
    {
        return *this;
    }

    if (image)
    {
        vmaDestroyImage(allocator, image, allocation);
        allocator = {};
        allocation = {};
        image = vk::Image{};
        layers = {};
    }

    swap(other);

    return *this;
}

UniqueVmaImage::~UniqueVmaImage()
{
    if (image)
    {
        vmaDestroyImage(allocator, image, allocation);
    }
}

void UniqueVmaImage::transfer_layout(vk::CommandBuffer cb,
    vk::ImageLayout src, vk::ImageLayout dst,
    vk::AccessFlags srcAccess, vk::AccessFlags dstAccess,
    vk::PipelineStageFlags srcStages, vk::PipelineStageFlags dstStages)
{
    vk::ImageMemoryBarrier barrier{
        srcAccess, dstAccess,
        src, dst,
        VK_QUEUE_FAMILY_IGNORED,
        VK_QUEUE_FAMILY_IGNORED,
        image,
        vk::ImageSubresourceRange{
            vk::ImageAspectFlagBits::eColor, 0, 1, 0, static_cast<uint32_t>(layers),
        }
    };
    cb.pipelineBarrier(
        srcStages, dstStages,
        {}, 0, nullptr, 0, nullptr, 1, &barrier);
}
