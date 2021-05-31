#include "InCoreClustering.hpp"

#include <queue>
#include <compare>
#include <numeric>
#include <stack>


#include "../DualSurfaceGraph.hpp"
#include "../SurfaceGraph.hpp"


std::vector<ThickTriangle> fixup_border(std::vector<ThickTriangle> triangles)
{
    DualSurfaceGraph dual_graph(triangles);
    SurfaceGraph graph(triangles);

    std::unordered_set<std::size_t> border_vertices;
    for (std::size_t i = 0; i < triangles.size(); ++i)
    {
        auto edges = triangle_edges(triangles[i]);
        for (const auto& edge : edges)
        {
            if (dual_graph.find_not(edge, i) == DualSurfaceGraph::INVALID)
            {
                border_vertices.emplace(graph.index_of(edge.first));
                border_vertices.emplace(graph.index_of(edge.second));
            }
        }
    }

    static constexpr std::size_t NONE = std::numeric_limits<std::size_t>::max();

    std::vector<std::size_t> colors(graph.vertex_count(), NONE);

    std::size_t color = 0;
    for (auto i : border_vertices)
    {
        if (colors[i] != NONE)
        {
            continue;
        }
        
        std::stack<std::size_t> vert_stack;
        vert_stack.push(i);

        while (!vert_stack.empty())
        {
            auto current = vert_stack.top();
            vert_stack.pop();

            if (colors[current] != NONE
                || !border_vertices.contains(current))
            {
                continue;
            }

            colors[current] = color;

            for (auto adj : graph.adjacent_to(current))
            {
                auto[f, s] = dual_graph.find(SymmetricEdge{graph.coords(current), graph.coords(adj)});
                if (s != DualSurfaceGraph::INVALID)
                {
                    continue;
                }

                vert_stack.push(adj);
            }
        }

        ++color;
    }


    std::unordered_set<SymmetricPair<std::size_t>> edges_to_split;

    for (auto i : border_vertices)
    {
        for (auto j : graph.adjacent_to(i))
        {
            auto[f, s] = dual_graph.find(SymmetricEdge{graph.coords(i), graph.coords(j)});

            if (s != NONE && colors[i] == colors[j])
            {
                edges_to_split.emplace(SymmetricPair<std::size_t>{i, j});
            }
        }
    }

    for (auto[i, j] : edges_to_split)
    {
        dual_graph.split_edge(triangles, graph.coords(i), graph.coords(j));
    }

    return triangles;
}

ClusteringData triangle_soup_to_clusters(const std::vector<ThickTriangle>& triangles)
{
    ClusteringData result;

    DualSurfaceGraph dual_graph{triangles};

    result.patches.reserve(triangles.size());

    for (auto& t : triangles)
    {
        auto idx = result.patches.size();
        result.patches.push_back({});
        auto& patch = result.patches.back();

        auto[e1, e2, e3] = triangle_edges(t);
        Vector3 vector_area;

        patch.perimeter = length(e1) + length(e2) + length(e3);
        {
            Vector4 v1 = projective_position(t.a) - projective_position(t.b);
            Vector4 v2 = projective_position(t.c) - projective_position(t.b);
            vector_area = v1.cross3(v2).block<3, 1>(0, 0);
        }
        patch.area = vector_area.norm() / 2;
        patch.vertex_count = 3;

        auto[v1, v2, v3] = triangle_verts(t);
        patch.boundary = {
            {dual_graph.find_not(e1, idx), length(e1), v1},
            {dual_graph.find_not(e2, idx), length(e2), v2},
            {dual_graph.find_not(e3, idx), length(e3), v3}};

        result.border_graph_vertices[v1].insert(idx);
        result.border_graph_vertices[v2].insert(idx);
        result.border_graph_vertices[v3].insert(idx);

        for (const auto& edge : patch.boundary)
        {
            if (edge.patch_idx == Patch::NONE)
            {
                result.border_graph_vertices[edge.starting_vertex].insert(Patch::NONE);
            }
        }

        auto make_planarity_quadric =
            [](const ThickVertex& v) -> Matrix4
            {
                auto u = projective_position(v);
                return u * u.transpose();
            };

        patch.planarity_quadric =
            make_planarity_quadric(t.a)
            + make_planarity_quadric(t.b)
            + make_planarity_quadric(t.c);

        {
            Vector4 tri_normal;
            tri_normal.block<3, 1>(0, 0) = -vector_area.normalized();
            tri_normal(3, 0) = 1;
            patch.orientation_quadric = tri_normal * tri_normal.transpose() * patch.area;
        }
    }

    for (auto& patch : result.patches)
    {
        for (auto& edge : patch.boundary)
        {
            if (result.border_graph_vertices[edge.starting_vertex].contains(Patch::NONE))
            {
                patch.has_vertices_adjacent_to_none = true;
                edge.starting_vertex_adjacent_to_none = true;
            }
        }
    }

    // Remove "pointy" triangles
    //       v those things have 2 neighbors
    // _____/\______/\___
    // there's nothing we can do about em.
    // after clusterization, a cluster might only have 1 neighboring cluster except for NONE,
    // therefore it's a degenerate 2-gon. We can't deal with those nicely and will have to do some trickery
    // to quadrangulate em.
    // TODO: think about alternate solution: enforce neighbor count >=3 constraint on clusters

    std::erase_if(result.border_graph_vertices,
        [](const auto& pair) { return pair.second.size() < 3; });

    result.accumulated_mapping.resize(triangles.size());
    std::iota(result.accumulated_mapping.begin(), result.accumulated_mapping.end(), 0);

    return result;
};

ClusteringData incore_cluster(const std::vector<ThickTriangle>& triangles, ClusteringMetricConfig metric_config,
    std::size_t target_memory, FloatingNumber max_error, FloatingNumber min_relative_cluster_count_change)
{
    ClusteringConfig config
    {
        metric_config,
        [target_memory, total = triangles.size(), min_relative_cluster_count_change, max_error]
        (FloatingNumber error, std::size_t patch_count, std::size_t memory)
        {
            return error < max_error
                && (memory > target_memory || patch_count > min_relative_cluster_count_change * total);
        }
    };

    return cluster(triangle_soup_to_clusters(triangles), std::move(config));
}
