#pragma once

#include <filesystem>

#include "Clustering.hpp"


void cluster_plain(const std::filesystem::path& plainfile, const std::filesystem::path& workdir,
    ClusteringMetricConfig metric_config, std::size_t memory_limit, FloatingNumber error_threshold);
