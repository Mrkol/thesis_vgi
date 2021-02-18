#pragma once

#include <filesystem>
#include <unordered_map>
#include <vector>
#include <Eigen/Dense>

#include "DataTypes.hpp"


// TODO: Allocators for speed
struct Patch
{
	Eigen::Matrix4f planarity_quadric;
	Eigen::Matrix4f orientation_quadric;

	// can be summed when stitching
	float area{};
	float perimeter{};
	std::size_t vertex_count{};

	constexpr static std::size_t NONE = std::numeric_limits<std::size_t>::max();
	struct BoundaryEdge
	{
		std::size_t patch_idx;
		float length;
	};

	std::vector<BoundaryEdge> boundary;
};

constexpr float quadric_weight = 1;
constexpr float compactness_weight = 1;


// Has O(n log n) complexity \o/
std::vector<Patch> cluster(std::filesystem::path plain);
