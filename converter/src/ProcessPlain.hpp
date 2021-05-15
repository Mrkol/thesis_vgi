#pragma once

#include <filesystem>

#include "clustering/Clustering.hpp"
#include "Parametrization.hpp"
#include "resampling/Resampler.hpp"


void process_plain(const std::filesystem::path& plainfile, const std::filesystem::path& workdir,
    const std::filesystem::path& output_dir, bool debug_output,
    ClusteringMetricConfig metric_config, std::size_t memory_limit, FloatingNumber error_threshold,
    ParametrizationConfig parametrization_config, ResamplerConfig resampler_config);
