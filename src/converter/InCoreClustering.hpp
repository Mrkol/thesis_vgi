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

	float area{};
	float perimeter{};
	std::size_t vertex_count{};

	constexpr static std::size_t NONE = std::numeric_limits<std::size_t>::max();
	struct BoundaryEdge
	{
		std::size_t patch_idx;
		float length;
		HashableCoords starting_vertice;
	};

	std::vector<BoundaryEdge> boundary;

	inline std::size_t total_size()
    {
	    return sizeof(Patch) + boundary.size() * sizeof(BoundaryEdge);
    }
};

constexpr float planarity_weight = 1;
constexpr float orientation_weight = 1;
constexpr float compactness_weight = 1;

std::tuple<std::vector<Patch>, std::vector<std::size_t>> cluster(std::filesystem::path plain);
