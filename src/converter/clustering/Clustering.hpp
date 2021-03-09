#pragma once

#include <unordered_map>
#include <vector>
#include <unordered_set>
#include <function2/function2.hpp>

#include "DataTypes.hpp"

struct ClusteringMetricConfig
{
    FloatingNumber planarity_weight = 1;
    FloatingNumber orientation_weight = 0;
    FloatingNumber compactness_weight = 1;
};

struct ClusteringConfig
{
    ClusteringMetricConfig metric_config;
    fu2::unique_function<bool(FloatingNumber /*error*/,
        std::size_t /*patch_count*/, std::size_t /*memory*/)> stopping_criterion;
};

// TODO: Allocators for speed
struct Patch
{
    Matrix4 planarity_quadric;
    Matrix4 orientation_quadric;

    FloatingNumber area{};
    FloatingNumber perimeter{};
    std::size_t vertex_count{};

    constexpr static std::size_t NONE = std::numeric_limits<std::size_t>::max();
    struct BoundaryEdge
    {
        std::size_t patch_idx;
        FloatingNumber length;
        HashableCoords starting_vertex;
        bool starting_vertex_adjacent_to_none{false};
    };

    std::vector<BoundaryEdge> boundary;
    bool has_vertices_adjacent_to_none{false};

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
    std::vector<std::size_t> accumulated_mapping;
};

void rotate_boundary(std::vector<Patch::BoundaryEdge>& merged_boundary);
void reindex_clustering_data(ClusteringData& data, const std::vector<std::size_t>& mapping);

struct IntersectionData
{
    FloatingNumber common_perimeter{0};
    std::size_t common_vertex_count{1};
};

FloatingNumber after_merge_error(const Patch& first, const Patch& second,
    const IntersectionData& data, ClusteringMetricConfig config);

ClusteringData cluster(ClusteringData data, ClusteringConfig config);

void check_consistency(const ClusteringData& data);
void check_topological_invariants(const ClusteringData& data);
