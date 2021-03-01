#pragma once

#include <filesystem>

#include "Clustering.hpp"


ClusteringData incore_cluster(const std::filesystem::path& plain, ClusteringMetricConfig metric_error,
    std::size_t target_memory, FloatingNumber max_error, FloatingNumber min_relative_cluster_count_change);
