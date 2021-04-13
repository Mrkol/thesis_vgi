#pragma once

#include <filesystem>
#include "../DataTypes.hpp"


struct QuadPatch
{
    std::vector<ThickTriangle> triangles;
    std::array<std::vector<HashableCoords>, 4> boundary;
};

std::vector<QuadPatch> quadrangulate(std::vector<ThickTriangle> triangles,
    std::size_t idx, const struct ClusteringData& data);
