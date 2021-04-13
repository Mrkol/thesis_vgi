#pragma once

#include <filesystem>

#include "Clustering.hpp"


ClusteringData triangle_soup_to_clusters(const std::vector<ThickTriangle>& triangles);

ClusteringData incore_cluster(const std::vector<ThickTriangle>& triangles, ClusteringMetricConfig metric_error,
    std::size_t target_memory, FloatingNumber max_error, FloatingNumber min_relative_cluster_count_change);
