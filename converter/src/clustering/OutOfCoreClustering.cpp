#include "OutOfCoreClustering.hpp"

#include <numeric>

#include "../DualSurfaceGraph.hpp"
#include "Clustering.hpp"


ClusteringData merge_clustering_data(std::vector<ClusteringData> datas)
{
#ifdef CLUSTERING_CONSISTENCY_CHECKS
    for (const auto& data : datas)
    {
        check_consistency(data);
        check_topological_invariants(data);
    }
#endif
    // Reindex separate datas to a common indexing
    {
        std::size_t already_mapped = 0;
        for (auto& data : datas)
        {
            std::vector<std::size_t> mapping;
            mapping.resize(data.patches.size());
            std::iota(mapping.begin(), mapping.end(), already_mapped);
            already_mapped += mapping.size();
            reindex_clustering_data(data, mapping);
        }
    }

    // Reconstruct cross-data adjacency information
    DualSurfaceGraph potential_common_edges;

    for (std::size_t current_idx = 0; const auto& data : datas)
    {
        for (auto& patch : data.patches)
        {
            if (patch.has_vertices_adjacent_to_none)
            {
                for (auto it = patch.boundary.begin(); it != patch.boundary.end(); ++it)
                {
                    auto next = std::next(it) == patch.boundary.end() ? patch.boundary.begin() : std::next(it);
                    if (it->patch_idx == Patch::NONE)
                    {
                        potential_common_edges.add({it->starting_vertex, next->starting_vertex}, current_idx);
                    }
                }
            }
            ++current_idx;
        }
    }

    std::size_t total_patch_count = 0;
    std::size_t total_triangle_count = 0;
    for (std::size_t current_idx = 0; auto& data : datas)
    {
        total_patch_count += data.patches.size();
        total_triangle_count += data.accumulated_mapping.size();
        for (auto& patch : data.patches)
        {
            if (patch.has_vertices_adjacent_to_none)
            {
                for (auto it = patch.boundary.begin(); it != patch.boundary.end(); ++it)
                {
                    auto next = std::next(it) == patch.boundary.end() ? patch.boundary.begin() : std::next(it);
                    if (it->patch_idx == Patch::NONE)
                    {
                        auto idx = potential_common_edges.find_not(
                            {it->starting_vertex, next->starting_vertex}, current_idx);

                        it->patch_idx = idx;
                    }
                }

                rotate_boundary(patch.boundary);
            }
            ++current_idx;
        }
    }

    ClusteringData merged;
    merged.patches.reserve(total_patch_count);
    merged.accumulated_mapping.reserve(total_triangle_count);
    for (auto& data : datas)
    {
        std::move(data.patches.begin(), data.patches.end(),
            std::back_inserter(merged.patches));
        std::move(data.accumulated_mapping.begin(), data.accumulated_mapping.end(),
            std::back_inserter(merged.accumulated_mapping));
        for (auto&[point, set] : data.border_graph_vertices)
        {
            merged.border_graph_vertices[point].insert(set.begin(), set.end());
        }
    }
    datas = {};

    // Reconstruct cross-data boundary graph vertices
    BorderGraphVertices cross_data_vertices;
    for (std::size_t idx = 0; const auto& patch : merged.patches)
    {
        if (patch.has_vertices_adjacent_to_none)
        {
            for (auto it = patch.boundary.begin(); it != patch.boundary.end(); ++it)
            {
                if (it->starting_vertex_adjacent_to_none)
                {
                    cross_data_vertices[it->starting_vertex].insert(idx);
                }

                auto next = std::next(it) == patch.boundary.end() ? patch.boundary.begin() : std::next(it);
                if (it->patch_idx == Patch::NONE)
                {
                    cross_data_vertices[it->starting_vertex].insert(Patch::NONE);
                    cross_data_vertices[next->starting_vertex].insert(Patch::NONE);
                }
            }
        }
        ++idx;
    }

    for (auto&[point, set] : cross_data_vertices)
    {
        if (set.size() >= 3)
        {
            merged.border_graph_vertices[point] = std::move(set);
        }
    }

    for (std::size_t idx = 0; auto& patch : merged.patches)
    {
        if (patch.has_vertices_adjacent_to_none)
        {
            patch.has_vertices_adjacent_to_none = false;
            for (auto it = patch.boundary.begin(); it != patch.boundary.end(); ++it)
            {
                auto prev = std::prev(it == patch.boundary.begin() ? patch.boundary.end() : it);

                it->starting_vertex_adjacent_to_none = it->patch_idx == Patch::NONE || prev->patch_idx == Patch::NONE;

                // for cases like
                // this    \/
                // _____________
                // /\/\/\/\/\/\/
                if (auto set_it = merged.border_graph_vertices.find(it->starting_vertex);
                    set_it != merged.border_graph_vertices.end() && set_it->second.contains(Patch::NONE))
                {
                    it->starting_vertex_adjacent_to_none = true;
                }

                patch.has_vertices_adjacent_to_none |= it->starting_vertex_adjacent_to_none;
            }
        }
        ++idx;
    }

#ifdef CLUSTERING_CONSISTENCY_CHECKS
    check_consistency(merged);
    check_topological_invariants(merged);
#endif

    return merged;
}

ClusteringData outofcore_cluster(std::vector<ClusteringData> incore_results,
    ClusteringMetricConfig metric_error, FloatingNumber max_error)
{
    auto data = merge_clustering_data(std::move(incore_results));

    ClusteringConfig config
    {
        metric_error,
        [max_error]
        (FloatingNumber error, std::size_t /*patch_count*/, std::size_t /*image_memory*/)
        {
            // TODO: Better criterion
            return error < max_error;
        }
    };

    return cluster(std::move(data), std::move(config));
}
