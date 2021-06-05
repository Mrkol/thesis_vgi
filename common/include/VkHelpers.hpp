#pragma once

#include <vulkan/vulkan.hpp>
#include <filesystem>
#include <fstream>


namespace VkHelpers
{

template<std::size_t size>
void check_validation_layers_support(const std::array<const char*, size>& validation_layers)
{
    if constexpr (size != 0)
    {
        auto layers = vk::enumerateInstanceLayerProperties();
        for (auto valid_layer : validation_layers)
        {
            bool found = false;
            for (auto layer : layers)
            {
                if (std::string_view(layer.layerName) == valid_layer)
                {
                    found = true;
                    break;
                }
            }

            if (!found)
            {
                throw std::runtime_error("Validation layer " + std::string(valid_layer) + " not supported!");
            }
        }
    }
}

}
