#pragma once

#include <filesystem>

#include "Clustering.hpp"


ClusteringData incore_cluster(std::filesystem::path plain, std::size_t target_memory);
