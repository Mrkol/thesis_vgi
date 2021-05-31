#pragma once

#include <vector>
#include <unordered_set>
#include <filesystem>
#include <Eigen/Dense>


struct GeometryImageVertex
{
    Eigen::Vector3f position;
    Eigen::Vector2f uv;
    Eigen::Vector3f normal;
};

static_assert(sizeof(GeometryImageVertex) == 8*sizeof(float));


/**
 * Only squares supported
 */
class GeometryImage
{
public:
    GeometryImage() = default;

    explicit GeometryImage(const std::filesystem::path& path);

    [[nodiscard]] GeometryImage upscale(std::size_t newsize) const;

    Eigen::Vector3f& operator() (std::size_t x, std::size_t y);
    const Eigen::Vector3f& operator() (std::size_t x, std::size_t y) const;
    [[nodiscard]] std::size_t size() const { return  width; }

    [[nodiscard]] const GeometryImageVertex* get_data() const { return data.data(); }

private:
    std::vector<GeometryImageVertex> data;
    std::size_t width{};
};
