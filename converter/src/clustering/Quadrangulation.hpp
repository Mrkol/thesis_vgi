#pragma once

#include <filesystem>
#include "../DataTypes.hpp"


void quadrangulate(const std::filesystem::path& patchfile, const std::filesystem::path& info_directory,
    std::size_t idx, const struct ClusteringData& data);
