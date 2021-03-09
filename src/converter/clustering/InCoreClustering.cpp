#include "InCoreClustering.hpp"

#include <fstream>
#include <queue>
#include <compare>
#include <numeric>
#include <iostream>

#include "SurfaceHashTable.hpp"
#include "DataTypes.hpp"


ClusteringData triangle_soup_to_clusters(const std::vector<ThickTriangle>& triangles)
{
    ClusteringData result;

    SurfaceHashTable<FloatingNumber> table;

    for (size_t i = 0; i < triangles.size(); ++i)
    {
        auto[e1, e2, e3] = triangle_edges(triangles[i]);
        table.add(e1, i);
        table.add(e2, i);
        table.add(e3, i);
    }

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
            {table.find_not(e1, idx), length(e1), v1},
            {table.find_not(e2, idx), length(e2), v2},
            {table.find_not(e3, idx), length(e3), v3}};

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

ClusteringData incore_cluster(const std::filesystem::path& plain, ClusteringMetricConfig metric_config,
    std::size_t target_memory, FloatingNumber max_error, FloatingNumber min_relative_cluster_count_change)
{
	std::vector<ThickTriangle> triangles;
	triangles.resize(file_size(plain) / sizeof(ThickTriangle));

	{
		std::ifstream in{plain, std::ios_base::binary};
        in.read(reinterpret_cast<char*>(triangles.data()), file_size(plain));
	}

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
