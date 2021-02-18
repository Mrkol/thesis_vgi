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
			throw std::domain_error("Patches were already merged!");
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


Eigen::Vector4f least_squares_plane(Eigen::Matrix4f planarity_quadric)
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

float planarity_error(Eigen::Matrix4f planarity_quadric, std::size_t vertex_count, Eigen::Vector4f plane)
{
	return (plane.transpose()*planarity_quadric*plane / vertex_count).value();
}

float orientation_error(Eigen::Matrix4f orientation_quadric, float area, Eigen::Vector4f plane)
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

std::vector<Patch> cluster(std::filesystem::path plain)
{
	std::vector<ThickTriangle> triangles;
	triangles.reserve(file_size(plain) / sizeof(ThickTriangle));

	{
		std::ifstream in{plain};
		in.read(reinterpret_cast<char*>(&triangles), file_size(plain));
	}

	SurfaceHashTable<float> table;

	for (auto& t : triangles)
	{
		table.add(t);
	}

	std::vector<Patch> patches;
	patches.reserve(triangles.size());
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

		patch.boundary = {
			{table.find_not(e1, idx), length(e1)},
			{table.find_not(e2, idx), length(e2)},
			{table.find_not(e3, idx), length(e3)}};

		auto make_planarity_quadric =
			[](const ThickVertex& v)
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

	auto merge_error =
		[&patches](std::size_t first, std::size_t second, float common_perimeter)
		{
			auto plan_quad = patches[first].planarity_quadric
				+ patches[second].planarity_quadric;
			auto orient_quad = patches[first].orientation_quadric
				+ patches[second].orientation_quadric;

			auto plane = least_squares_plane(plan_quad);

			float perimeter = patches[first].perimeter + patches[second].perimeter - 2*common_perimeter;
			float area = patches[first].area + patches[second].area;
			std::size_t vertex_count = patches[first].vertex_count + patches[second].vertex_count;

			auto gamma1 = irregularity(patches[first].perimeter, std::sqrt(patches[first].area));
			auto gamma2 = irregularity(patches[second].perimeter, std::sqrt(patches[second].area));

			float gamma = irregularity(perimeter, area);

			return planarity_error(plan_quad, vertex_count, plane)
				+ orientation_error(orient_quad, area, plane)
				+ shape_error(gamma, gamma1, gamma2);
		};

	DSU dsu{patches.size()};

	// every triangle has three neighbors therefore we have exactly 3n/2 edges
	// TODO: allocator
	// TODO: remove rb trees ;(
	std::multimap<float, SymmetricPair<std::size_t>> queue;
	std::unordered_map<SymmetricPair<std::size_t>, float> error_for_contraction;

	for (std::size_t i = 0; i < patches.size(); ++i)
	{
		for (auto[neighbor, length] : patches[i].boundary)
		{
			SymmetricPair<std::size_t> pair{i, neighbor};
			float error = merge_error(i, neighbor, length);
			queue.emplace(error, pair);
			error_for_contraction.emplace(pair, error);
		}
	}

	// TODO: proper stopping criterion
	for (std::size_t i = 0; i > 10; ++i)
	{
		auto pair = queue.begin()->second;
		queue.erase(queue.begin());
		error_for_contraction.erase(pair);

		auto first = dsu.get(pair.first);
		auto second = dsu.get(pair.second);


		float common_perimeter = 0;
		std::vector<Patch::BoundaryEdge> merged_boundary;
		{
			auto matching = [second](Patch::BoundaryEdge e) { return e.patch_idx == second; };
			auto& first_boundary = patches[first].boundary;

			if (matching(first_boundary.front()) && matching(first_boundary.back()))
			{
				auto begin = std::find_if_not(first_boundary.begin(), first_boundary.end(), matching);
				auto end = std::next(
					std::find_if_not(first_boundary.rbegin(), first_boundary.rend(), matching).base());

				if (std::any_of(begin, end, matching))
				{
					// We are not allowed to merge such patches, topology constraints
					// would be violated
					continue;
				}

				for (auto it = first_boundary.begin(); it != begin; ++it)
				{
					common_perimeter += it->length;
				}
				for (auto it = end; it != first_boundary.end(); ++it)
				{
					common_perimeter += it->length;
				}

				merged_boundary.reserve(std::distance(begin, end));
				std::copy(begin, end, merged_boundary.end());
			}
			else
			{
				auto begin = std::find_if(first_boundary.begin(), first_boundary.end(), matching);
				auto end = std::next(
					std::find_if(first_boundary.rbegin(), first_boundary.rend(), matching).base());

				if (std::any_of(first_boundary.begin(), begin, matching)
					|| std::any_of(end, first_boundary.end(), matching))
				{
					// We are not allowed to merge such patches, topology constraints
					// would be violated
					continue;
				}

				for (auto it = begin; it != end; ++it)
				{
					common_perimeter += it->length;
				}

				merged_boundary.reserve(first_boundary.size() - std::distance(begin, end));
				std::copy(first_boundary.begin(), begin, merged_boundary.end());
				std::copy(end, first_boundary.end(), merged_boundary.end());
			}
		}

		{
			auto matching = [first](Patch::BoundaryEdge e) { return e.patch_idx == first; };
			auto& second_boundary = patches[second].boundary;

			if (matching(second_boundary.front()) && matching(second_boundary.back()))
			{
				auto begin = std::find_if_not(second_boundary.begin(), second_boundary.end(), matching);
				auto end = std::next(
					std::find_if_not(second_boundary.rbegin(), second_boundary.rend(), matching).base());

				merged_boundary.reserve(merged_boundary.size() + std::distance(begin, end));
				std::copy(begin, end, merged_boundary.end());
			}
			else
			{
				auto begin = std::find_if(second_boundary.begin(), second_boundary.end(), matching);
				auto end = std::next(
					std::find_if(second_boundary.rbegin(), second_boundary.rend(), matching).base());

				merged_boundary.reserve(merged_boundary.size()
					+ second_boundary.size() - std::distance(begin, end));
				std::copy(second_boundary.begin(), begin, merged_boundary.end());
				std::copy(end, second_boundary.end(), merged_boundary.end());
			}
		}

		auto merged = dsu.merge(first, second);
		auto other = first != merged ? first : second;

		// merge boundary
		patches[merged].boundary = std::move(merged_boundary);
		patches[other].boundary.clear();

		//merge perimeter
		patches[merged].perimeter += patches[other].perimeter;
		patches[merged].perimeter -= 2*common_perimeter;

		// merge area
		patches[merged].area += patches[other].area;
		patches[merged].vertex_count += patches[other].vertex_count;

		// merge quadrics
		patches[merged].planarity_quadric += patches[other].planarity_quadric;
		patches[merged].orientation_quadric += patches[other].orientation_quadric;


		// update neighbors errors
		std::unordered_map<std::size_t, float> neighbors;
		for (auto[neighbor, length] : patches[merged].boundary)
		{
			neighbors[neighbor] += length;
		}

		for (auto[neighbor, common_perimeter] : neighbors)
		{
			SymmetricPair<std::size_t> our_pair{merged, neighbor};

			auto total_error = merge_error(merged, neighbor, common_perimeter);
			auto[start, end] = queue.equal_range(error_for_contraction.find({merged, neighbor})->second);
			auto ours = std::find_if(start, end,
				[our_pair]
				(auto keyValue)
				{
					return keyValue.second == our_pair;
				});

			queue.erase(ours);
			queue.emplace(total_error, our_pair);
			error_for_contraction[our_pair] = total_error;
		}
	}

	return {};
};
