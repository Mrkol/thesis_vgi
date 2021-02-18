#pragma once

#include <tuple>
#include <cmath>
#include <Eigen/Dense>

#include "TupleHash.hpp"
#include "SymmetricPair.hpp"

using HashableCoords = std::tuple<float, float, float>;

using SymmetricEdge = SymmetricPair<HashableCoords>;

inline float length(const SymmetricEdge& edge)
{
	auto sqr = [](float x) { return x*x; };
	return std::sqrt(sqr(std::get<0>(edge.first) - std::get<0>(edge.second))
		+ sqr(std::get<1>(edge.first) - std::get<1>(edge.second))
		+ sqr(std::get<2>(edge.first) - std::get<2>(edge.second)));
}

struct ThickVertex
{
	float x, y, z;
	float nx, ny, nz;
	float u, v, w;
};

inline Eigen::Vector4f projective_position(const ThickVertex& v)
{
	return {v.x, v.y, v.z, 1};
}

inline Eigen::Vector4f projective_normal(const ThickVertex& v)
{
	return {v.x, v.y, v.z, 0};
}

inline HashableCoords to_hashable_coords(const ThickVertex& vert)
{
	return {vert.x, vert.y, vert.z};
}

struct ThickTriangle
{
	ThickVertex a;
	ThickVertex b;
	ThickVertex c;
};

static_assert(sizeof(ThickTriangle) == 9*4*3);

inline std::tuple<SymmetricEdge, SymmetricEdge, SymmetricEdge> triangle_edges(const ThickTriangle& tri)
{
	auto[a, b, c] = tri;
	auto a_ = to_hashable_coords(a);
	auto b_ = to_hashable_coords(b);
	auto c_ = to_hashable_coords(c);
	return {{a_, b_}, {b_, c_}, {c_, a_}};
}

struct Dimensions
{
	float min_x, max_x;
	float min_y, max_y;
	float min_z, max_z;
};

inline float MeanDimension(Dimensions d)
{
	return (d.max_x + d.max_y + d.max_z - d.min_x - d.min_y - d.min_z) / 3;
}
