#pragma once

#include <unordered_map>
#include <vector>
#include <Eigen/Dense>
#include <unordered_set>

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
    bool has_adjacent_nones{};

    [[nodiscard]] inline std::size_t total_size() const
    {
        return sizeof(Patch) + boundary.size() * sizeof(BoundaryEdge);
    }
};

// All patch borders form a planar graph vertices of which need to be tracked during clustering
using BorderGraphVertices = std::unordered_map<HashableCoords, std::unordered_set<std::size_t>>;

struct ClusteringData
{
    std::vector<Patch> patches;
    BorderGraphVertices border_graph_vertices;
};

void reindex_clustering_data(ClusteringData& data, const std::vector<std::size_t>& mapping);

constexpr float planarity_weight = 1;
constexpr float orientation_weight = 1;
constexpr float compactness_weight = 1;

float after_merge_error(const Patch& first, const Patch& second, float common_perimeter);

ClusteringData cluster(ClusteringData data, std::function<bool(float, std::size_t, std::size_t)> stopping_criterion);

