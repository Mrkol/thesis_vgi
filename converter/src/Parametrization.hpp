#pragma once

#include <filesystem>
#include "DataTypes.hpp"
#include "clustering/Quadrangulation.hpp"

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

struct MappingElement
{
    HashableCoords key;
    MappingCoords value;
};

std::vector<MappingElement> parametrize(const QuadPatch& patch, const ParametrizationConfig& config);

