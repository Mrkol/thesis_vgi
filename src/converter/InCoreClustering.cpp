#include "InCoreClustering.hpp"

#include <fstream>
#include <queue>
#include <compare>
#include <algorithm>
#include <numeric>
#include <map>
#include <unordered_map>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <set>
#include <unordered_set>
#include <iostream>

#include "SurfaceHashTable.hpp"
#include "DataTypes.hpp"



class DSU
{
	std::size_t& parent_of(std::size_t idx)
	{
		return elements[idx].replacement_idx;
	}

public:
	DSU(std::size_t size)
	{
		elements.reserve(size);
		for (size_t i = 0; i < size; ++i)
		{
			elements.push_back({i, 1});
		}
	}

	std::size_t get(std::size_t idx)
	{
		while (elements[idx].replacement_idx != idx)
		{
			std::size_t tmp = parent_of(idx);
			parent_of(idx) = parent_of(parent_of(idx));
			idx = tmp;
		}

		return idx;
	}

	// returns new representative
	std::size_t merge(std::size_t i, std::size_t j)
	{
		i = get(i);
		j = get(j);

		if (i == j)
		{
			throw std::logic_error("Patches were already merged!");
		}

		if (elements[i].size < elements[j].size)
		{
			std::swap(i, j);
		}

		elements[j].replacement_idx = i;
		elements[i].size += elements[j].size;

		return i;
	}

private:
	struct Element
	{
		std::size_t replacement_idx;
		std::size_t size;
	};

	std::vector<Element> elements;
};


Eigen::Vector4f least_squares_plane(const Eigen::Matrix4f& planarity_quadric)
{
	Eigen::Matrix3f A = planarity_quadric.block<3, 3>(0, 0);
	Eigen::Vector3f b = planarity_quadric.block<3, 1>(0, 3) * 2;
	float c = planarity_quadric(3, 3);
	Eigen::Matrix3f Z = A - b*b.transpose()/c;

	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver;
	solver.compute(Z);

	Eigen::Vector3f least_squares_plane_normal = solver.eigenvectors().col(0);

	float least_squares_plane_offset = -least_squares_plane_normal.dot(b)/c;

	Eigen::Vector4f result;
	result.block<3, 1>(0, 0) = least_squares_plane_normal;
	result(3, 0) = least_squares_plane_offset;
	return result;
}

float planarity_error(const Eigen::Matrix4f& planarity_quadric, std::size_t vertex_count, const Eigen::Vector4f& plane)
{
	return (plane.transpose()*planarity_quadric*plane / vertex_count).value();
}

float orientation_error(const Eigen::Matrix4f& orientation_quadric, float area, const Eigen::Vector4f& plane)
{
	return (plane.transpose()*orientation_quadric*plane / area).value();
}

float irregularity(float perimeter, float area)
{
	return perimeter*perimeter / (4*M_PI*area);
}

float shape_error(float gamma, float gamma1, float gamma2)
{
	return (gamma - std::max(gamma1, gamma2))/gamma;
}

std::tuple<std::vector<Patch>, std::vector<std::size_t>> cluster(std::filesystem::path plain)
{
	std::vector<ThickTriangle> triangles;
	triangles.resize(file_size(plain) / sizeof(ThickTriangle));

	{
		std::ifstream in{plain};
        in.read(reinterpret_cast<char*>(triangles.data()), file_size(plain));
	}

	SurfaceHashTable<float> table;

	for (auto& t : triangles)
	{
		table.add(t);
	}

	std::vector<Patch> patches;
	patches.reserve(triangles.size());
    std::size_t total_patches_size = 0;

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

		total_patches_size += patch.total_size();
	}
	// TODO: Free triangle memory

	auto merge_error =
		[&patches](std::size_t first, std::size_t second, float common_perimeter)
		{
			Eigen::Matrix4f plan_quad = patches[first].planarity_quadric
				+ patches[second].planarity_quadric;
            Eigen::Matrix4f orient_quad = patches[first].orientation_quadric
				+ patches[second].orientation_quadric;

			auto plane = least_squares_plane(plan_quad);

			float perimeter = patches[first].perimeter + patches[second].perimeter - 2*common_perimeter;
			float area = patches[first].area + patches[second].area;
			std::size_t vertex_count = patches[first].vertex_count + patches[second].vertex_count;

			auto gamma1 = irregularity(patches[first].perimeter, std::sqrt(patches[first].area));
			auto gamma2 = irregularity(patches[second].perimeter, std::sqrt(patches[second].area));

			float gamma = irregularity(perimeter, area);

			return planarity_weight * planarity_error(plan_quad, vertex_count, plane)
				+ orientation_weight * orientation_error(orient_quad, area, plane)
				+ compactness_weight * shape_error(gamma, gamma1, gamma2);
		};

	DSU dsu{patches.size()};

	// TODO: allocator or remove rb trees ;(
	std::multimap<float, SymmetricPair<std::size_t>> queue;
	std::unordered_map<SymmetricPair<std::size_t>, decltype(queue)::const_iterator> position_in_queue;
    position_in_queue.reserve(3*patches.size());

	for (std::size_t i = 0; i < patches.size(); ++i)
	{
		for (auto[neighbor, length, _] : patches[i].boundary)
		{
		    if (neighbor == Patch::NONE || neighbor > i)
            {
		        continue;
            }
			SymmetricPair<std::size_t> pair{i, neighbor};
			float error = merge_error(i, neighbor, length);
            position_in_queue.emplace(pair, queue.emplace(error, pair));
		}
	}

    auto remove_old_contraction = [&position_in_queue, &queue](SymmetricPair<std::size_t> contr)
        {
            auto it = position_in_queue.find(contr);
            if (it == position_in_queue.end())
            {
                // This means that the contraction got rejected due to topological invariant violation.
                return;
                //throw std::logic_error("Neighboring patches were not queued up for contraction!");
            }
            queue.erase(it->second);
            position_in_queue.erase(it);
        };

	for (std::size_t current_patch_count = patches.size();
	    current_patch_count > 10 && total_patches_size >= 4096 && !queue.empty();) {
        auto[first, second] = queue.begin()->second;

        if (first != dsu.get(first) || second != dsu.get(second)) {
            throw std::logic_error("Invariant violated!");
        }

        remove_old_contraction({first, second});

        auto update_boundary =
                [&dsu](std::vector<Patch::BoundaryEdge> &boundary) {
                    for (auto&[neighbor, _, __] : boundary) {
                        if (neighbor != Patch::NONE) {
                            neighbor = dsu.get(neighbor);
                        }
                    }
                };

        update_boundary(patches[first].boundary);
        update_boundary(patches[second].boundary);

        // Build common boundary and check for topological invariant
        std::vector<Patch::BoundaryEdge> merged_boundary;
        std::size_t double_common_vertex_count = 0;
        float double_common_perimeter = 0;

        HashableCoords first_endpoint;
        HashableCoords second_endpoint;
        {
            //TODO: dont capture so much stuff
            auto process_boundary =
                [&patches, &merged_boundary, &double_common_perimeter,
                    &double_common_vertex_count]
                (std::size_t patch, std::size_t other)
                {
                    auto matching = [other](Patch::BoundaryEdge e) { return e.patch_idx == other; };
                    const auto &boundary = patches[patch].boundary;

                    auto begin = std::find_if(boundary.begin(), boundary.end(), matching);
                    auto end = std::find_if(boundary.rbegin(), boundary.rend(), matching).base();

                    if (begin == boundary.end() || end == boundary.begin())
                    {
                        throw std::logic_error("Trying to merge non-neighboring patches!");
                    }

                    for (auto it = begin; it != end; ++it)
                    {
                        double_common_perimeter += it->length;
                    }

                    merged_boundary.reserve(boundary.size() - std::distance(begin, end));
                    std::copy(end, boundary.end(), std::back_inserter(merged_boundary));
                    std::copy(boundary.begin(), begin, std::back_inserter(merged_boundary));
                    double_common_vertex_count += std::distance(begin, end) + 1;

                    auto end_ = end == boundary.end() ? boundary.begin() : end;
                    return std::tuple(begin->starting_vertice, end_->starting_vertice);
                };

            {
                auto[a, b] = process_boundary(first, second);
                auto[c, d] = process_boundary(second, first);

                if (a != d || b != c)
                {
                    throw std::logic_error("Touching vertices calculation is messed up");
                }

                first_endpoint = a;
                second_endpoint = b;
            }

            // Fixup the boundary so that other code is simpler
            if (merged_boundary.front().patch_idx == merged_boundary.back().patch_idx)
            {
                auto patch = merged_boundary.front().patch_idx;
                auto it = std::find_if(merged_boundary.begin(), merged_boundary.end(),
                    [patch](const Patch::BoundaryEdge& e){ return e.patch_idx != patch; });
                std::rotate(merged_boundary.begin(), it, merged_boundary.end());
            }
        }

        {
            // Topological constraint: all patch intersections must be be either empty, a point, or
            // homeomorphic to a segment. If this merge results in something else, dont do it.

            bool two_single_point_intersections = false;

            std::unordered_set<std::size_t> single_point_intersections;
            for (auto it = merged_boundary.begin(); !two_single_point_intersections
                && it != merged_boundary.end(); ++it)
            {
                auto prev = it == merged_boundary.begin() ? std::prev(merged_boundary.end()) : std::prev(it);

                auto patches = border_graph_vertices.find(it->starting_vertice);
                if (patches != border_graph_vertices.end())
                {
                    // Approximately constant time due to typical models not
                    // having verts with high numbers of adjacent edges
                    for (auto patch : patches->second)
                    {
                        if (patch != first && patch != second && patch != Patch::NONE
                            && patch != it->patch_idx && patch != prev->patch_idx)
                        {
                            if (single_point_intersections.contains(patch))
                            {
                                two_single_point_intersections = true;
                                break;
                            }
                            single_point_intersections.insert(patch);
                        }
                    }
                }
            }

            // Some other patch touches both first and second by a single point, therefore the merge is invalid.
            if (two_single_point_intersections)
            {
                continue;
            }

            bool more_than_a_segment = false;
            std::unordered_set<std::size_t> already_met;
            for (std::size_t i = 0; i < merged_boundary.size(); )
            {
                auto current = merged_boundary[i].patch_idx;

                if (current == Patch::NONE)
                {
                    ++i;
                    continue;
                }

                if (already_met.contains(current) || single_point_intersections.contains(current))
                {
                    more_than_a_segment = true;
                    break;
                }

                while (merged_boundary[i].patch_idx == current) { ++i; }
                already_met.insert(current);
            }

            if (more_than_a_segment)
            {
                continue;
            }
        }

		{
			auto matching = [first = first](Patch::BoundaryEdge e) { return e.patch_idx == first; };
			auto& second_boundary = patches[second].boundary;

			if (matching(second_boundary.front()) && matching(second_boundary.back()))
			{
				auto begin = std::find_if_not(second_boundary.begin(), second_boundary.end(), matching);
				auto end = std::find_if_not(second_boundary.rbegin(), second_boundary.rend(), matching).base();

				merged_boundary.reserve(merged_boundary.size() + std::distance(begin, end));
				std::copy(begin, end, std::back_inserter(merged_boundary));
			}
			else
			{
				auto begin = std::find_if(second_boundary.begin(), second_boundary.end(), matching);
				auto end = std::find_if(second_boundary.rbegin(), second_boundary.rend(), matching).base();

                if (begin == second_boundary.end() || end == second_boundary.begin())
                {
                    throw std::logic_error("Trying to merge non-neighboring patches!");
                }

				merged_boundary.reserve(merged_boundary.size()
					+ second_boundary.size() - std::distance(begin, end));
                std::copy(end, second_boundary.end(), std::back_inserter(merged_boundary));
                std::copy(second_boundary.begin(), begin, std::back_inserter(merged_boundary));
			}
		}

        {
            std::array pair{first, second};

            for (size_t i = 0; i < pair.size(); ++i)
            {
                std::unordered_set<std::size_t> neighbors;
                for (auto[idx, _, ___] : patches[pair[i]].boundary)
                {
                    if (idx != Patch::NONE && idx != pair[1 - i])
                    {
                        neighbors.insert(idx);
                    }
                }

                for (auto neighbor : neighbors)
                {
                    remove_old_contraction({pair[i], neighbor});
                }
            }
        }

		auto merged = dsu.merge(first, second);
		auto other = first != merged ? first : second;

        {
            for (auto point : {first_endpoint, second_endpoint})
            {
                auto it = border_graph_vertices.find(point);
                if (it == border_graph_vertices.end())
                {
                    throw std::logic_error("Border graph vertices was missing!");
                }
                it->second.erase(first);
                it->second.erase(second);

                if (it->second.empty())
                {
                    border_graph_vertices.erase(it);
                }
                else
                {
                    it->second.insert(merged);
                }
            }
        }

		--current_patch_count;
		total_patches_size -= patches[first].total_size() + patches[second].total_size();

		// merge boundary
		patches[merged].boundary = std::move(merged_boundary);
		patches[other].boundary.clear();

		//merge perimeter
		patches[merged].perimeter += patches[other].perimeter;
		patches[merged].perimeter -= double_common_perimeter;

		// merge area
		patches[merged].area += patches[other].area;

        patches[merged].vertex_count += patches[other].vertex_count;
        patches[merged].vertex_count -= double_common_vertex_count;

		// merge quadrics
		patches[merged].planarity_quadric += patches[other].planarity_quadric;
		patches[merged].orientation_quadric += patches[other].orientation_quadric;

		// Recalculate neighbor merge errors
        {
            std::unordered_map<std::size_t, float> neighbors;
            for (auto[neighbor, length, _] : patches[merged].boundary)
            {
                if (neighbor != Patch::NONE)
                {
                    neighbors[neighbor] += length;
                }
            }

            for (auto[neighbor, common_perimeter] : neighbors)
            {
                SymmetricPair<std::size_t> our_pair{merged, neighbor};
                auto total_error = merge_error(merged, neighbor, common_perimeter);

                position_in_queue.emplace(our_pair, queue.emplace(total_error, our_pair));
            }
        }

        total_patches_size += patches[merged].total_size();
		patches[other] = {};
	}

    std::vector<std::size_t> resulting_patch_ids;
    {
        std::set<std::size_t> resulting_patch_ids_set;
        for (std::size_t i = 0; i < patches.size(); ++i)
        {
            resulting_patch_ids_set.insert(dsu.get(i));
        }
        resulting_patch_ids.reserve(resulting_patch_ids_set.size());
        for (auto idx : resulting_patch_ids_set)
        {
            resulting_patch_ids.push_back(idx);
        }
    }

    std::vector<std::size_t> triangle_to_patch;
	triangle_to_patch.resize(patches.size(), 0);
	for (std::size_t i = 0; i < patches.size(); ++i)
    {
        triangle_to_patch[i] = std::lower_bound(resulting_patch_ids.begin(), resulting_patch_ids.end(), dsu.get(i))
                - resulting_patch_ids.begin();
    }

	std::vector<Patch> resulting_patches;
	for (auto idx : resulting_patch_ids)
    {
	    resulting_patches.push_back(std::move(patches[idx]));
    }

	return {std::move(resulting_patches), std::move(triangle_to_patch)};
};
