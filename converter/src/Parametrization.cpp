#include "Parametrization.hpp"

#include <unordered_map>
#include <iostream>

#include "DataTypes.hpp"
#include "SurfaceGraph.hpp"


using AdjacentTriangles = std::vector<std::unordered_set<std::size_t>>;

void optimize_uniform_springs(const SurfaceGraph& graph, const std::unordered_set<std::size_t>& border_indices,
    std::vector<Vector2>& result, const UniformSpringOptimizerConfig& config)
{
    std::vector<Vector2> gradients{result.size()};
    FloatingNumber total_gradient_norm = std::numeric_limits<FloatingNumber>::max();
    while (total_gradient_norm > config.gradient_threshold)
    {
        for (size_t idx = 0; idx < graph.vertex_count(); ++idx)
        {
            if (!border_indices.contains(idx))
            {
                // gradient of uniform spring system (sum over edges ||e||^2)
                gradients[idx] = result[idx] * graph.adjacent_to(idx).size();
                for (auto neighbor : graph.adjacent_to(idx))
                {
                    gradients[idx] -= result[neighbor];
                }
            }
            else
            {
                // bounadry condition
                gradients[idx] = {0, 0};
            }
        }

        total_gradient_norm = 0;
        for (size_t i = 0; i < graph.vertex_count(); ++i)
        {
            total_gradient_norm = std::max(total_gradient_norm, gradients[i].norm());
            result[i] -= config.descent_rate * gradients[i];
        }
    }
}

void initialize_boundary(const SurfaceGraph& graph, const std::array<std::vector<HashableCoords>, 4>& edges,
    std::vector<Vector2>& result)
{
    std::array<std::vector<FloatingNumber>, 4> prefix_sums;
    for (size_t i = 0; i < 4; ++i)
    {
        prefix_sums[i].resize(edges[i].size(), 0);
        for (size_t j = 1; j < edges[i].size(); ++j)
        {
            prefix_sums[i][j] = prefix_sums[i][j - 1] + length({edges[i][j - 1], edges[i][j]});
        }
    }

    /**
     *        Unit square
     *  x ->
     * y         <- 3
     * |   ________________
     * V  |               |
     *    |               | ^
     * 0  |               | |
     * |  |               | 2
     * V  |               |
     *    |_______________|
     *         1 ->
     */
    for (size_t i = 0; i < 4; ++i)
    {
        for (size_t j = 0; j < edges[i].size(); ++j)
        {
            FloatingNumber pos = prefix_sums[i][j] / prefix_sums[i].back();
            auto idx = graph.index_of(edges[i][j]);

            FloatingNumber first = i == 1 || i == 2 ? 1 : 0;
            FloatingNumber second = i == 0 || i == 1 ? pos : 1 - pos;

            if (i == 1 || i == 3)
            {
                std::swap(first, second);
            }

            result[idx] = {first, second};
        }
    }
}

std::array<Vector3, 3> to_eigen(std::array<HashableCoords, 3> coords)
{
    std::array<Vector3, 3> result;
    for (std::size_t i = 0; i < coords.size(); ++i)
    {
        auto[x, y, z] = coords[i];
        result[i] << x, y, z;
    }
    return result;
}

using Jacobian = Eigen::Matrix<FloatingNumber, 3, 2>;

Jacobian parametrization_jacobian(const SurfaceGraph& graph, const std::vector<Vector2>& mapping,
    const ThickTriangle& triangle)
{
    // See stretch energy by Sander et al
    auto[a, b, c] =  triangle_verts(triangle);
    auto[p1, p2, p3] = std::array{mapping[graph.index_of(a)], mapping[graph.index_of(b)], mapping[graph.index_of(c)]};
    auto[q1, q2, q3] = to_eigen({a, b, c});

    FloatingNumber double_area_signed;
    {
        Matrix2 m;
        m << (p2 - p1), (p3 - p1);
        double_area_signed = m.determinant();
    }


    Jacobian result = q1*(p2 - p3).transpose() + q2*(p3 - p1).transpose() + q3*(p1 - p2).transpose();

    Matrix2 flip;
    flip << 0,-1,
            1, 0;
    result *= flip;

    result /= double_area_signed;

    return result;
}

FloatingNumber stretch_L2_squared(const Jacobian& J)
{
    FloatingNumber a = J.col(0).squaredNorm();
    FloatingNumber b = J.col(1).squaredNorm();
    return (a + b)/2;
}

bool triangle_orientation(const SurfaceGraph& graph, const std::vector<ThickTriangle>& triangles,
    const std::vector<Vector2>& result, std::size_t idx)
{
    auto[a, b, c] = triangle_verts(triangles[idx]);
    auto[p1, p2, p3] = std::array{
        result[graph.index_of(a)], result[graph.index_of(b)], result[graph.index_of(c)]};

    Matrix2 m;
    m << (p2 - p1), (p3 - p1);

    return m.determinant() < 0;
}

template<class Iter>
FloatingNumber total_stretch_L2(const SurfaceGraph& graph, const std::vector<Vector2>& mapping,
    const std::vector<ThickTriangle>& triangles, const std::vector<bool>& original_triangle_orientations,
    Iter indices_begin, Iter indices_end)
{
    FloatingNumber numerator = 0;
    FloatingNumber denominator = 0;
    for (auto it = indices_begin; it != indices_end; ++it)
    {
        std::size_t idx = *it;
        const auto& tri = triangles[idx];
        auto[q1, q2, q3] = to_eigen(triangle_verts(tri));

        if (original_triangle_orientations[idx] != triangle_orientation(graph, triangles, mapping, idx))
        {
            return std::numeric_limits<FloatingNumber>::infinity();
        }

        FloatingNumber stretch_squared = stretch_L2_squared(parametrization_jacobian(graph, mapping, tri));
        FloatingNumber area = (q2 - q1).cross(q3 - q1).norm() / 2;

        denominator += area;
        numerator += stretch_squared * area;
    }
    return std::sqrt(numerator / denominator);
}

void optimize_L2_stretch(const SurfaceGraph& graph, const AdjacentTriangles& triangles_adjacent_to_vert,
    const std::vector<ThickTriangle>& triangles, const std::unordered_set<std::size_t>& border_indices,
    std::vector<Vector2>& mapping, const StretchOptimizerConfig& config)
{
    std::vector<bool> original_triangle_orientations;
    original_triangle_orientations.resize(triangles.size());
    FloatingNumber normalization_constant = 0;
    for (std::size_t idx = 0; idx < triangles.size(); ++idx)
    {
        original_triangle_orientations[idx] = triangle_orientation(graph, triangles, mapping, idx);
        auto[a, b, c] = to_eigen(triangle_verts(triangles[idx]));
        normalization_constant += (b - a).cross(c - a).norm();
    }
    normalization_constant = std::sqrt(normalization_constant);


    std::vector<std::size_t> iota{triangles.size()};
    std::iota(iota.begin(), iota.end(), 0);

    auto neighborhood_stretch_L2 =
        [&graph, &triangles, &triangles_adjacent_to_vert, &mapping, &original_triangle_orientations,
         &normalization_constant]
        (std::size_t idx)
        {
            return total_stretch_L2(graph, mapping, triangles, original_triangle_orientations,
                triangles_adjacent_to_vert[idx].begin(), triangles_adjacent_to_vert[idx].end())
                    / normalization_constant;
        };

    std::vector<std::pair<FloatingNumber, size_t>> optimization_order;
    optimization_order.reserve(graph.vertex_count());
    for (size_t idx = 0; idx < graph.vertex_count(); ++idx)
    {
        if (!border_indices.contains(idx))
        {
            optimization_order.emplace_back(neighborhood_stretch_L2(idx), idx);
        }
    }
    std::sort(optimization_order.begin(), optimization_order.end(), std::greater{});

    FloatingNumber root_of_vert_count = std::sqrt(graph.vertex_count());

    for (size_t i = 1; i <= config.max_iterations; ++i)
    {
        const auto search_extents = FloatingNumber{1}/i/root_of_vert_count;

        FloatingNumber error_change = 0;
        for (auto&[stretch, idx] : optimization_order)
        {
            Vector2 direction;
            direction.setRandom();
            direction.normalize();

            Vector2 old = mapping[idx];

            FloatingNumber left = -search_extents;
            FloatingNumber right = search_extents;
            while (right - left > 1e-1/i)
            {
                FloatingNumber onethird = std::lerp(left, right, FloatingNumber{1}/3);
                mapping[idx] = old + onethird * direction;
                FloatingNumber onethird_stretch = neighborhood_stretch_L2(idx);

                FloatingNumber twothirds = std::lerp(left, right, FloatingNumber{2}/3);
                mapping[idx] = old + twothirds * direction;
                FloatingNumber twothirds_stretch = neighborhood_stretch_L2(idx);

                if (std::isinf(onethird_stretch) && std::isinf(twothirds_stretch))
                {
                    right = twothirds;
                    left = onethird;
                }

                if (onethird_stretch < twothirds_stretch)
                {
                    right = twothirds;
                }
                else
                {
                    left = onethird;
                }
            }

            mapping[idx] = old + std::midpoint(left, right) * direction;
            auto new_stretch = neighborhood_stretch_L2(idx);
            // If we are already exactly in the minimum point, result of ternary search won't be
            // better than our previous result
            if (new_stretch < stretch)
            {
                error_change += (stretch - new_stretch)*(stretch - new_stretch);
                stretch = new_stretch;
            }
            else
            {
                mapping[idx] = old;
            }
        }

        auto tsh = config.average_local_stretch_difference_threshold;
        if (error_change / optimization_order.size() < tsh*tsh)
        {
            break;
        }

        std::sort(optimization_order.begin(), optimization_order.end(), std::greater{});
    }
}

void parametrize(const std::filesystem::path& patch, const std::filesystem::path& info_dir,
    const ParametrizationConfig& config)
{
    auto triangles = read_plainfile(patch);

    SurfaceGraph graph{triangles};
    AdjacentTriangles triangles_adjacent_to_vert{graph.vertex_count()};

    for (std::size_t idx = 0; idx < triangles.size(); ++idx)
    {
        for (auto vert : triangle_verts(triangles[idx]))
        {
            triangles_adjacent_to_vert[graph.index_of(vert)].insert(idx);
        }
    }

    std::unordered_set<std::size_t> border_indices;

    std::vector<Vector2> result;
    result.resize(graph.vertex_count(), {0.5, 0.5});

    {
        auto patch_info = info_dir / patch.filename();

        std::array<std::vector<HashableCoords>, 4> edges;
        {
            std::ifstream info{patch_info, std::ios_base::binary};
            for (auto& edge : edges)
            {
                edge = read_vector<HashableCoords>(info);

                for (auto v : edge)
                {
                    border_indices.insert(graph.index_of(v));
                }
            }
        }

        initialize_boundary(graph, edges, result);

        remove(patch_info);
    }

    optimize_uniform_springs(graph, border_indices, result,
        config.uniform_spring_optimizer_config);

    optimize_L2_stretch(graph, triangles_adjacent_to_vert, triangles, border_indices, result,
        config.stretch_optimizer_config);


    {
        std::ofstream out{info_dir / patch.filename().string(), std::ios_base::binary};

        for (std::size_t i = 0; i < graph.vertex_count(); ++i)
        {
            auto M = graph.coords(i);
            MappingCoords m {result[i].x(), result[i].y()};
            out.write(reinterpret_cast<char*>(&M), sizeof(M));
            out.write(reinterpret_cast<char*>(&m), sizeof(m));
        }
    }
}
