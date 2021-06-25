#pragma once

#include <memory>
#include <vector>
#include <vulkan/vulkan.hpp>


class Shader : public std::enable_shared_from_this<Shader>
{
public:
    Shader(vk::Device device, std::string_view name);

    void reload(vk::Device device);

    [[nodiscard]] vk::ShaderModule get() const { return shader_module_.get(); }

private:
    std::string name_;
    std::vector<std::byte> data_;
    vk::UniqueShaderModule shader_module_;
};

using ShaderPtr = std::shared_ptr<Shader>;
