#include "UniqueStbImage.hpp"

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

UniqueStbImage::UniqueStbImage(const std::filesystem::path& path)
{
    int x, y, n;
    auto d = stbi_load(path.string().c_str(), &x, &y, &n, 4);
    if (d == nullptr)
    {
        throw std::runtime_error("Unable to load texture!");
    }
    data_ = reinterpret_cast<std::byte*>(d);
    width_ = x;
    height_ = y;
    pixel_size_ = n;
}

void UniqueStbImage::swap(UniqueStbImage& other)
{
    std::swap(data_, other.data_);
    std::swap(width_, other.width_);
    std::swap(height_, other.height_);
    std::swap(pixel_size_, other.pixel_size_);
}

UniqueStbImage::UniqueStbImage(UniqueStbImage&& other) noexcept
{
    swap(other);
}

UniqueStbImage& UniqueStbImage::operator=(UniqueStbImage&& other) noexcept
{
    if (this == &other)
    {
        return *this;
    }

    if (data_)
    {
        stbi_image_free(data_);
        data_ = nullptr;
        width_ = 0;
        height_ = 0;
        pixel_size_ = 0;
    }

    swap(other);

    return *this;
}

UniqueStbImage::~UniqueStbImage() noexcept
{
    stbi_image_free(data_);
}
