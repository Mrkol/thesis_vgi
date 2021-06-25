#pragma once

#include <vulkan/vulkan.hpp>
#include <memory>
#include <filesystem>
#include <span>

#include "UniqueVmaImage.hpp"


class Texture : public std::enable_shared_from_this<Texture>
{
public:
    explicit Texture(std::span<const std::filesystem::path> layers, class IResourceManager* irm);

    vk::ImageView get() const { return textures_view_.get(); }

private:
    UniqueVmaImage textures_;
    vk::UniqueImageView textures_view_;
};

using TexturePtr = std::shared_ptr<Texture>;
