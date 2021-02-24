#pragma once

#include <filesystem>

#include "Clustering.hpp"


ClusteringData incore_cluster(const std::filesystem::path& plain, std::size_t target_memory);
