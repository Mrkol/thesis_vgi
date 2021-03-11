#pragma once

#include <tuple>
#include <cmath>
#include <array>
#include <numeric>
#include <Eigen/Dense>

#include "TupleHash.hpp"
#include "SymmetricPair.hpp"


using FloatingNumber = double;

using HashableCoords = std::tuple<FloatingNumber, FloatingNumber, FloatingNumber>;
static_assert(sizeof(HashableCoords) == sizeof(FloatingNumber)*3);

using SymmetricEdge = SymmetricPair<HashableCoords>;

inline FloatingNumber length(const SymmetricEdge& edge)
{
	auto sqr = [](FloatingNumber x) { return x*x; };
	return std::sqrt(sqr(std::get<0>(edge.first) - std::get<0>(edge.second))
		+ sqr(std::get<1>(edge.first) - std::get<1>(edge.second))
		+ sqr(std::get<2>(edge.first) - std::get<2>(edge.second)));
}

struct ThickVertex
{
    FloatingNumber x {}, y {}, z {};
    FloatingNumber nx{}, ny{}, nz{};
    FloatingNumber u {}, v {}, w {};
};

using Vector4 = Eigen::Matrix<FloatingNumber, 4, 1>;
using Matrix4 = Eigen::Matrix<FloatingNumber, 4, 4>;
using Vector3 = Eigen::Matrix<FloatingNumber, 3, 1>;
using Matrix3 = Eigen::Matrix<FloatingNumber, 3, 3>;

inline ThickVertex midpoint(const ThickVertex& a, const ThickVertex& b)
{
    return
    {
        std::midpoint(a.x, b.x), std::midpoint(a.y, b.y), std::midpoint(a.z, b.z),
        std::midpoint(a.nx, b.nx), std::midpoint(a.ny, b.ny), std::midpoint(a.nz, b.nz),
        std::midpoint(a.u, b.u), std::midpoint(a.v, b.v), std::midpoint(a.w, b.w),
    };
}

inline Vector4 projective_position(const ThickVertex& v)
{
	return {v.x, v.y, v.z, 1};
}

inline Vector4 projective_normal(const ThickVertex& v)
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

static_assert(sizeof(ThickTriangle) == sizeof(FloatingNumber)*9*3);

inline std::array<HashableCoords, 3> triangle_verts(const ThickTriangle& tri)
{
    auto[a, b, c] = tri;
    return {to_hashable_coords(a), to_hashable_coords(b), to_hashable_coords(c)};
}

inline std::array<SymmetricEdge, 3> triangle_edges(const ThickTriangle& tri)
{
    auto[a, b, c] = triangle_verts(tri);
    return {SymmetricEdge{a, b}, {b, c}, {c, a}};
}

struct Dimensions
{
	FloatingNumber min_x{}, max_x{};
    FloatingNumber min_y{}, max_y{};
    FloatingNumber min_z{}, max_z{};
};

inline FloatingNumber MeanDimension(Dimensions d)
{
	return (d.max_x + d.max_y + d.max_z - d.min_x - d.min_y - d.min_z) / 3;
}
