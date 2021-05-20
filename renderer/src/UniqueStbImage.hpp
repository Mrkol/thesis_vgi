#pragma once

#include <filesystem>
#include <stb_image.h>


class UniqueStbImage
{
public:
    UniqueStbImage() = default;
    explicit UniqueStbImage(const std::filesystem::path& path);

    UniqueStbImage(const UniqueStbImage&) = delete;
    UniqueStbImage& operator=(const UniqueStbImage&) = delete;

    UniqueStbImage(UniqueStbImage&&) noexcept;
    UniqueStbImage& operator=(UniqueStbImage&&) noexcept;

    void swap(UniqueStbImage& other);

    ~UniqueStbImage() noexcept;

    std::byte* data() { return data_; }
    [[nodiscard]] std::size_t width() const { return width_; }
    [[nodiscard]] std::size_t height() const { return height_; }
    [[nodiscard]] std::size_t pixel_size() const { return pixel_size_; }

    [[nodiscard]] std::size_t size() const { return width_ * height_ * pixel_size_; }


private:
    std::byte* data_{nullptr};
    std::size_t width_{0};
    std::size_t height_{0};
    std::size_t pixel_size_{0};
};
