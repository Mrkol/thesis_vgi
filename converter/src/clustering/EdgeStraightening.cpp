#include "EdgeStraightening.hpp"

#include "../SurfaceGraph.hpp"
#include "../DualSurfaceGraph.hpp"

std::vector<Patch::BoundaryEdge> extract_boundary_part(
    const std::vector<Patch::BoundaryEdge>& boundary, std::size_t idx)
{
    auto matches = [idx](Patch::BoundaryEdge edge) { return edge.patch_idx == idx; };
    auto start = std::find_if(boundary.begin(), boundary.end(), matches);
    auto end = std::find_if_not(start, boundary.end(), matches);

    std::vector<Patch::BoundaryEdge> result;
    result.reserve(std::distance(boundary.begin(), start) + std::distance(end, boundary.end()));
    std::copy(boundary.begin(), start, std::back_inserter(result));
    std::copy(end, boundary.end(), std::back_inserter(result));
    return result;
}

template<class T>
std::vector<T> concat(const std::vector<T>& first, const std::vector<T>& second)
{
    std::vector<T> result;
    result.reserve(first.size() + second.size());
    std::copy(first.begin(), first.end(), std::back_inserter(result));
    std::copy(second.begin(), second.end(), std::back_inserter(result));

    return result;
}

void straighten_edges(ClusterWithIndex first, ClusterWithIndex second, const ClusteringData& clustering_data)
{
    auto first_extracted = extract_boundary_part(clustering_data.patches[first.idx].boundary, second.idx);
    auto second_extracted = extract_boundary_part(clustering_data.patches[second.idx].boundary, first.idx);

    auto merged_triangles = concat(first.cluster, second.cluster);
    SurfaceGraph graph{merged_triangles};

    std::vector<FloatingNumber> distances_to_boundary(graph.vertex_count(),
        std::numeric_limits<FloatingNumber>::max());

    auto start = graph.index_of(first_extracted.front().starting_vertex);
    auto end = graph.index_of(second_extracted.front().starting_vertex);

    for (std::size_t i = 0; i < graph.vertex_count(); ++i)
    {
        auto& min = distances_to_boundary[i];

        auto process = [&min, &graph, i, start, end](auto& boundary)
            {
                for (auto& edge : boundary)
                {
                    auto v = graph.index_of(edge.starting_vertex);
                    if (v == start || v == end)
                    {
                        continue;
                    }
                    min = std::min(min, graph.get_distance(v, i));
                }
            };
        process(first_extracted);
        process(second_extracted);
    }

    auto straightened_path = graph.scaled_a_star(start, end,
        [&distances_to_boundary](std::size_t j) { return distances_to_boundary[j]; });

    // Updating the clustering data here is pretty hard, but its not really used anywhere after this point,
    // so we'll simply leave it out of date.
    // TODO: maybe do it just in case

    DualSurfaceGraph dual_graph{merged_triangles};
    std::unordered_set<SymmetricEdge> banned;
    for (std::size_t i = 0; i < straightened_path.size() - 1; ++i)
    {
        banned.insert({graph.coords(straightened_path[i]), graph.coords(straightened_path[i + 1])});
    }

    auto colors = dual_graph.paint_quads(merged_triangles, banned);

    first.cluster.clear();
    second.cluster.clear();

    for (std::size_t idx = 0; idx < merged_triangles.size(); ++idx)
    {
        if (colors[idx] == 0)
        {
            first.cluster.push_back(merged_triangles[idx]);
        }
        else
        {
            second.cluster.push_back(merged_triangles[idx]);
        }
    }
}
