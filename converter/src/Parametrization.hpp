#pragma once

#include <filesystem>
#include "DataTypes.hpp"


struct StretchOptimizerConfig
{
    FloatingNumber average_local_stretch_difference_threshold = 1e-6;
    std::size_t max_iterations = 100;
};

struct UniformSpringOptimizerConfig
{
    FloatingNumber descent_rate = 1e-2;
    FloatingNumber gradient_threshold = 1e-4;
};

struct ParametrizationConfig
{
    UniformSpringOptimizerConfig uniform_spring_optimizer_config;
    StretchOptimizerConfig stretch_optimizer_config;

};

void parametrize(const std::filesystem::path& patch, const std::filesystem::path& info_dir,
    const ParametrizationConfig& config);

