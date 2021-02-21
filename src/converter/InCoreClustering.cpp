#include "InCoreClustering.hpp"

#include <fstream>
#include <queue>
#include <compare>
#include <numeric>
#include <unordered_map>
#include <Eigen/Geometry>
#include <unordered_set>
#include <iostream>

#include "SurfaceHashTable.hpp"
#include "DataTypes.hpp"




ClusteringData incore_cluster(std::filesystem::path plain, std::size_t target_memory)
{
	std::vector<ThickTriangle> triangles;
	triangles.resize(file_size(plain) / sizeof(ThickTriangle));

	{
		std::ifstream in{plain};
        in.read(reinterpret_cast<char*>(triangles.data()), file_size(plain));
	}

	SurfaceHashTable<float> table;

	for (std::size_t i = 0; i < triangles.size(); ++i)
	{
	    auto[e1, e2, e3] = triangle_edges(triangles[i]);
        table.add(e1, i);
        table.add(e2, i);
        table.add(e3, i);
	}

	std::vector<Patch> patches;
	patches.reserve(triangles.size());

    // Endpoints of patch intersections
    std::unordered_map<HashableCoords, std::unordered_set<std::size_t>> border_graph_vertices;

    for (auto& t : triangles)
	{
		auto idx = patches.size();
		patches.push_back({});
		auto& patch = patches.back();

        auto[e1, e2, e3] = triangle_edges(t);
		Eigen::Vector3f vector_area;

		patch.perimeter = length(e1) + length(e2) + length(e3);
		{
			Eigen::Vector4f v1 = projective_position(t.a) - projective_position(t.b);
			Eigen::Vector4f v2 = projective_position(t.c) - projective_position(t.b);
			vector_area = v1.cross3(v2).block<3, 1>(0, 0);
		}
		patch.area = vector_area.norm();
		patch.vertex_count = 3;

        auto[v1, v2, v3] = triangle_verts(t);
		patch.boundary = {
			{table.find_not(e1, idx), length(e1), v1},
			{table.find_not(e2, idx), length(e2), v2},
			{table.find_not(e3, idx), length(e3), v3}};
        border_graph_vertices[v1].insert(idx);
        border_graph_vertices[v2].insert(idx);
        border_graph_vertices[v3].insert(idx);

		auto make_planarity_quadric =
			[](const ThickVertex& v) -> Eigen::Matrix4f
			{
				auto u = projective_position(v);
				return u * u.transpose();
			};

		patch.planarity_quadric =
			make_planarity_quadric(t.a)
			+ make_planarity_quadric(t.b)
			+ make_planarity_quadric(t.c);

		{
			Eigen::Vector4f tri_normal;
			tri_normal.block<3, 1>(0, 0) = vector_area.normalized();
			tri_normal(3, 0) = -1;
			patch.orientation_quadric = tri_normal * tri_normal.transpose() * patch.area;
		}
	}

	auto stopping_criterion =
	    [target_memory, total = patches.size()](float error, std::size_t patch_count, std::size_t memory)
        {
            return memory > target_memory || 4*patch_count > total;
        };

	return cluster({std::move(patches), std::move(border_graph_vertices)}, stopping_criterion);
};
