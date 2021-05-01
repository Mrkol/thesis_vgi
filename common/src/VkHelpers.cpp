#include "VkHelpers.hpp"

namespace VkHelpers
{

std::vector<char> read_shader(std::string_view name)
{
    auto path = std::filesystem::current_path();
    path /= "shaders";
    path /= name.data() + std::string(".spv");
    std::ifstream file{path, std::ios::ate | std::ios::binary};

    if (!file.is_open())
    {
        throw std::runtime_error("Shader is missing!");
    }

    std::vector<char> result(file.tellg());
    file.seekg(0);
    file.read(result.data(), static_cast<std::streamsize>(result.size()));

    return result;
}

uint32_t find_memory_type(const vk::PhysicalDevice& physical_device,
    uint32_t typeFilter, const vk::MemoryPropertyFlags& flags)
{
    auto properties = physical_device.getMemoryProperties();

    for (uint32_t i = 0; i < properties.memoryTypeCount; ++i)
    {
        if (typeFilter & (1 << i)
            && (properties.memoryTypes[i].propertyFlags & flags) == flags)
        {
            return i;
        }
    }

    throw std::runtime_error("Couldn't find an appropriate image_memory type!");
}

}
