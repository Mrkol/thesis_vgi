#pragma once

#include <vector>

#include "../DataTypes.hpp"
#include "Clustering.hpp"


struct ClusterWithIndex
{
    std::vector<ThickTriangle>& cluster;
    std::size_t idx;
};

void straighten_edges(ClusterWithIndex first, ClusterWithIndex second, ClusteringData& clustering_data);
