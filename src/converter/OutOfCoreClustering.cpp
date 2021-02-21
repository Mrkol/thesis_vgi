#include "OutOfCoreClustering.hpp"

#include <numeric>

#include "SurfaceHashTable.hpp"


ClusteringData merge_clustering_data(std::vector<ClusteringData> datas)
{
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
    SurfaceHashTable<float> potential_common_edges;
    BorderGraphVertices potential_border_graph_vertices;

    for (std::size_t current_idx = 0; const auto& data : datas)
    {
        for (auto& patch : data.patches)
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
            ++current_idx;
        }
    }

    std::size_t total_patch_count = 0;
    for (std::size_t current_idx = 0; auto& data : datas)
    {
        total_patch_count += data.patches.size();
        for (auto& patch : data.patches)
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
            ++current_idx;
        }
    }

    ClusteringData merged;
    merged.patches.reserve(total_patch_count);
    for (auto& data : datas)
    {
        std::move(data.patches.begin(), data.patches.end(),
            std::back_inserter(merged.patches));
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

#ifndef NDEBUG

    BorderGraphVertices vert_checker;
    SurfaceHashTable<float> edge_checker;
    for (std::size_t idx = 0; idx < merged.patches.size(); ++idx)
    {
        const auto& patch = merged.patches[idx];
        for (auto it = patch.boundary.begin(); it != patch.boundary.end(); ++it)
        {
            auto next = std::next(it) == patch.boundary.end() ? patch.boundary.begin() : std::next(it);
            vert_checker[it->starting_vertice].insert(idx);
            vert_checker[next->starting_vertice].insert(idx);
            edge_checker.add({it->starting_vertice, next->starting_vertice}, idx);
        }
    }

    for (auto&[point, set] : vert_checker)
    {
        if (set.size() >= 3
            && (!merged.border_graph_vertices.contains(point)
                || merged.border_graph_vertices[point] != set))
        {
            throw std::logic_error("Merging algorithm is broken!");
        }
    }

    for (auto&[edge, pair] : edge_checker)
    {
        auto check = [&, e = edge](std::size_t idx, std::size_t neighbor)
            {
                auto& boundary = merged.patches[idx].boundary;
                std::size_t found = 0;
                while (found != boundary.size())
                {
                    auto next = found + 1 == boundary.size() ? 0 : found + 1;
                    if (e == SymmetricEdge{boundary[found].starting_vertice, boundary[next].starting_vertice})
                    {
                        break;
                    }
                }

                if (found == boundary.size()
                    || boundary[found].patch_idx != neighbor)
                {
                    throw std::logic_error("Merging algorithm is broken!");
                }
            };

        check(pair.first, pair.second);
        check(pair.second, pair.first);
    }

#endif

    return merged;
}

ClusteringData outofcore_cluster(std::vector<ClusteringData> incore_results)
{
    auto data = merge_clustering_data(std::move(incore_results));

    auto stopping_criterion =
        [](float error, std::size_t patch_count, std::size_t memory)
        {
            // TODO: Better criterion
            return error < 1e7;
        };

    return cluster(data, stopping_criterion);
}
