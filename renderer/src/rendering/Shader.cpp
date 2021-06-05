#include "Shader.hpp"

#include <filesystem>
#include <fstream>


Shader::Shader(vk::Device device, std::string_view name)
    : name_{name}
{
    reload(device);
}

void Shader::reload(vk::Device device)
{
    shader_module_ = {};
    data_.clear();

    auto path = std::filesystem::current_path() / "shaders" / (name_ + ".spv");
    std::ifstream file{path, std::ios::ate | std::ios::binary};

    if (!file.is_open())
    {
        throw std::runtime_error("Shader is missing!");
    }

    data_.resize(file.tellg());
    file.seekg(0);
    file.read(reinterpret_cast<char*>(data_.data()), static_cast<std::streamsize>(data_.size()));

    shader_module_ = device.createShaderModuleUnique(vk::ShaderModuleCreateInfo{
        {},
        data_.size(), reinterpret_cast<const uint32_t*>(data_.data())
    });
}
