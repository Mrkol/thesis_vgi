#pragma once

#include <filesystem>
#include <function2/function2.hpp>

class OutOfCoreTexture
{
public:
    void request_patch(std::size_t logarithmic_size, std::size_t offset_x, std::size_t offset_y,
        fu2::unique_function<void()>);
private:
    std::size_t bytes_per_piexel;
    std::size_t total_size;
    std::size_t index;
    class OutOfCoreTexture* owner;
};

/**
 * Manager for out of core textures.
 */
class OutOfCoreTextureManager
{
public:

private:

};

