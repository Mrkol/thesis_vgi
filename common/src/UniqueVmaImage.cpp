#include "UniqueVmaImage.hpp"


UniqueVmaImage::UniqueVmaImage(VmaAllocator allocator, vk::Format format, vk::Extent2D extent,
    vk::ImageTiling tiling, vk::ImageUsageFlags image_usage, VmaMemoryUsage memory_usage, uint32_t layers)
    : allocator{allocator}
{
    vk::ImageCreateInfo image_info{
        {}, vk::ImageType::e2D, format, vk::Extent3D{extent.width, extent.height, 1},
        1,
        layers,
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

UniqueVmaImage::UniqueVmaImage(UniqueVmaImage&& other) noexcept
{
    allocator = other.allocator;
    allocation = other.allocation;
    image = other.image;
    other.allocator = {};
    other.allocation = {};
    other.image = vk::Image{};
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
    }

    allocator = other.allocator;
    allocation = other.allocation;
    image = other.image;
    other.allocator = {};
    other.allocation = {};
    other.image = vk::Image{};

    return *this;
}

UniqueVmaImage::~UniqueVmaImage()
{
    if (image)
    {
        vmaDestroyImage(allocator, image, allocation);
    }
}
