#include "OutOfCoreClustering.hpp"

#include <numeric>

#include "SurfaceHashTable.hpp"
#include "Clustering.hpp"


ClusteringData merge_clustering_data(std::vector<ClusteringData> datas)
{
#ifdef CLUSTERING_CONSISTENCY_CHECKS
    for (const auto& data : datas)
    {
        check_consistency(data);
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
#ifdef CLUSTERING_CONSISTENCY_CHECKS
    for (const auto& data : datas)
    {
        check_consistency(data);
    }
#endif

    // Reconstruct cross-data adjacency information
    SurfaceHashTable<float> potential_common_edges;
    BorderGraphVertices potential_border_graph_vertices;

    for (std::size_t current_idx = 0; const auto& data : datas)
    {
        for (auto& patch : data.patches)
        {
            if (patch.has_adjacent_nones)
            {
                for (auto it = patch.boundary.begin(); it != patch.boundary.end(); ++it)
                {
                    auto next = std::next(it) == patch.boundary.end() ? patch.boundary.begin() : std::next(it);
                    if (it->patch_idx == Patch::NONE)
                    {
                        potential_common_edges.add({it->starting_vertice, next->starting_vertice}, current_idx);
                        potential_border_graph_vertices[it->starting_vertice].insert(current_idx);
                        potential_border_graph_vertices[next->starting_vertice].insert(current_idx);
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
            if (patch.has_adjacent_nones)
            {
                for (auto it = patch.boundary.begin(); it != patch.boundary.end(); ++it)
                {
                    auto next = std::next(it) == patch.boundary.end() ? patch.boundary.begin() : std::next(it);
                    if (it->patch_idx == Patch::NONE)
                    {
                        auto idx = potential_common_edges.find_not(
                            {it->starting_vertice, next->starting_vertice}, current_idx);

                        it->patch_idx = idx;
                    }
                }
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
    for (auto&[point, set] : potential_border_graph_vertices)
    {
        if (set.size() >= 3)
        {
            merged.border_graph_vertices[point].insert(set.begin(), set.end());
        }
    }

#ifdef CLUSTERING_CONSISTENCY_CHECKS
    check_consistency(merged);
#endif

    return merged;
}

ClusteringData outofcore_cluster(std::vector<ClusteringData> incore_results)
{
    auto data = merge_clustering_data(std::move(incore_results));

    auto stopping_criterion =
        [](float error, std::size_t /*patch_count*/, std::size_t /*memory*/)
        {
            // TODO: Better criterion
            return error < 1e7;
        };

    return cluster(std::move(data), stopping_criterion);
}
