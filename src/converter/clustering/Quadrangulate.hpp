#pragma once

#include <filesystem>


void quadrangulate(const std::filesystem::path& patchfile, std::size_t idx, const struct ClusteringData& data);
