#include "Quadrangulation.hpp"

#include <vector>
#include <fstream>
#include <unordered_map>
#include <queue>
#include <stack>

#include "Clustering.hpp"
#include "../DualSurfaceGraph.hpp"
#include "../SurfaceGraph.hpp"


std::vector<std::size_t>::const_iterator median_point(const SurfaceGraph& graph,
      const std::vector<std::size_t>& path)
{
    if (path.empty())
    {
        throw std::logic_error("Empty paths dont have median points!");
    }

    FloatingNumber total_length = 0;
    for (std::size_t i = 1; i < path.size(); ++i)
    {
        total_length += graph.get_distance(path[i], path[i - 1]);
    }

    auto it = path.begin();
    FloatingNumber current = 0;
    while (std::next(it) != path.end())
    {
        auto potential = current + graph.get_distance(*it, *std::next(it));
        if (std::abs(total_length / 2 - current) < std::abs(total_length / 2 - potential))
        {
            break;
        }
        current = potential;
        ++it;
    }

    return it;
}

std::vector<std::size_t> find_center_recurse(const SurfaceGraph& graph, const std::vector<std::size_t>& corners)
{
    if (corners.size() < 3)
    {
        throw std::logic_error("Midpoint recursion not possible with < 3 corners");
    }

    std::size_t m = *median_point(graph, graph.a_star(corners[0], corners[1]));

    auto distances = graph.dijkstra(m);

    std::vector<std::size_t> result;
    result.reserve(corners.size());

    for (std::size_t i = 2; i < corners.size(); ++i)
    {
        result.push_back(*median_point(graph, graph.recontstruct_shortest_path(distances, m, corners[i])));
    }

    return result;
}

using PolygonEdges = std::vector<std::vector<std::size_t>>;


void center_finding_dfs(const SurfaceGraph& graph, std::unordered_set<std::size_t>& candidates,
    std::size_t current, std::size_t depth = 0)
{
    if (candidates.contains(current) || depth > 3)
    {
        return;
    }

    candidates.insert(current);

    for (auto next : graph.adjacent_to(current))
    {
        center_finding_dfs(graph, candidates, next, depth + 1);
    }
}

std::size_t find_center(const SurfaceGraph& graph, const PolygonEdges& polygon_edges)
{
    std::unordered_set<std::size_t> border;
    for (auto& edge : polygon_edges)
    {
        std::copy(edge.begin(), edge.end(), std::inserter(border, border.begin()));
    }


    std::vector<std::size_t> current;
    current.reserve(polygon_edges.size());
    for (auto& edge : polygon_edges)
    {
        current.push_back(edge.front());
    }

    while (current.size() > 2)
    {
        current = find_center_recurse(graph, current);
    }

    std::size_t naive_candidate;
    if (current.size() == 2)
    {
        naive_candidate = *median_point(graph, graph.a_star(current[0], current[1]));
    }
    else
    {
        naive_candidate = current[0];
    }


    std::unordered_set<std::size_t> candidates;
    center_finding_dfs(graph, candidates, naive_candidate);

    static constexpr std::size_t NOT_FOUND = std::numeric_limits<std::size_t>::max();
    std::size_t best = NOT_FOUND;
    for (auto candidate : candidates)
    {
        if (!border.contains(candidate))
        {
            if (best == NOT_FOUND || graph.adjacent_to(best).size() < graph.adjacent_to(candidate).size())
            {
                best = candidate;
            }
        }
    }

    if (best == NOT_FOUND)
    {
        throw std::logic_error("fml");
    }

    return best;
}

void split_edge(std::vector<ThickTriangle>& patch, SurfaceGraph& graph, DualSurfaceGraph& dual_graph,
    size_t u_idx, size_t v_idx)
{
    // TODO: Optimize this crap
    auto u = graph.coords(u_idx);
    auto v = graph.coords(v_idx);

    /*       a                            a
     *      /\                           /|\
     *     /  \                         / | \
     *    / f  \                       / f|f'\
     * u /______\        =>         u /___|___\
     *   \      / v                   \   |   / v
     *    \  s /                       \s'|s /
     *     \  /                         \ | /
     *      \/                           \|/
     *      b                             b
     */

    // f, s
    auto[first_idx, second_idx] = dual_graph.find({u, v});

    if (first_idx == Patch::NONE)
    {
        throw std::logic_error("Trying to split an edge that does not belong to any triangle!");
    }

    auto get_third =
        [](ThickTriangle& triangle, SymmetricEdge e) -> ThickVertex&
        {
            if (SymmetricEdge{to_hashable_coords(triangle.a), to_hashable_coords(triangle.b)} == e)
            { return triangle.c; }
            if (SymmetricEdge{to_hashable_coords(triangle.b), to_hashable_coords(triangle.c)} == e)
            { return triangle.a; }
            if (SymmetricEdge{to_hashable_coords(triangle.c), to_hashable_coords(triangle.a)} == e)
            { return triangle.b; }
            throw std::logic_error("Edge didn't come from this triangle!");
        };

    HashableCoords a = to_hashable_coords(get_third(patch[first_idx], {u, v}));
    HashableCoords b{};
    if (second_idx != Patch::NONE)
    {
        b = to_hashable_coords(get_third(patch[second_idx], {u, v}));
    }


    // Split actual geometric data
    ThickVertex m_thick;
    size_t first_prime_idx;
    size_t second_prime_idx;
    {
        m_thick = midpoint(get_third(patch[first_idx], {a, v}), get_third(patch[first_idx], {a, u}));

        // Duplicate first and shift u -> m
        first_prime_idx = patch.size();
        patch.push_back(patch[first_idx]);
        get_third(patch.back(), {a, v}) = m_thick;

        if (second_idx != Patch::NONE)
        {
            // Duplicate second and shift v -> m
            second_prime_idx = patch.size();
            patch.push_back(patch[second_idx]);
            get_third(patch.back(), {b, u}) = m_thick;
        }

        // For first, shift v -> m
        get_third(patch[first_idx], {a, u}) = m_thick;

        if (second_idx != Patch::NONE)
        {
            // for second, shift u -> m
            get_third(patch[second_idx], {b, v}) = m_thick;
        }
    }

    auto m = to_hashable_coords(m_thick);

    // Split hash table data
    // Don't try to read this. Look at the picture above.
    {
        dual_graph.remove_triangle(a, u, v);
        if (second_idx != Patch::NONE)
        {
            dual_graph.remove_triangle(u, v, b);
        }

        dual_graph.add({a, v}, first_prime_idx);
        if (second_idx != Patch::NONE)
        {
            dual_graph.add({v, b}, second_idx);
            dual_graph.add({b, u}, second_prime_idx);
        }
        dual_graph.add({u, a}, first_idx);

        dual_graph.add({a, m}, first_idx);
        dual_graph.add({a, m}, first_prime_idx);
        dual_graph.add({v, m}, first_prime_idx);
        if (second_idx != Patch::NONE)
        {
            dual_graph.add({v, m}, second_idx);
            dual_graph.add({b, m}, second_idx);
            dual_graph.add({b, m}, second_prime_idx);
            dual_graph.add({u, m}, second_prime_idx);
        }
        dual_graph.add({u, m}, first_idx);
    }

    // Split graph
    graph.split_edge(u, v, a, second_idx != Patch::NONE ? std::make_optional(b) : std::nullopt, m);
}

std::vector<std::vector<std::size_t>> build_midpoint_paths(
    std::vector<ThickTriangle>& patch, SurfaceGraph& graph, DualSurfaceGraph& dual_graph,
    size_t center, PolygonEdges& polygon_edges)
{
    // First we have to split edges that are only 2 verts in length
    for (auto& edge : polygon_edges)
    {
        if (edge.size() == 2)
        {
            auto u = edge[0];
            auto v = edge[1];
            edge = {u, graph.vertex_count(), v};
            split_edge(patch, graph, dual_graph, u, v);
        }
    }


    // This hardcoded epsilon might be a bad idea
    static constexpr FloatingNumber NUMERICAL_STABILITY_EPS = 1e-6;

    std::vector<PolygonEdges::value_type::const_iterator> midpoints;
    midpoints.reserve(polygon_edges.size());
    for (const auto& edge : polygon_edges)
    {
        midpoints.push_back(median_point(graph, edge));
    }

    std::vector<std::vector<size_t>> result{midpoints.size()};

    auto check_not_border_edge =
        [&graph, &dual_graph](std::size_t u, std::size_t v)
        {
            if (dual_graph.find({graph.coords(u), graph.coords(v)}).second == Patch::NONE)
            {
                throw std::logic_error("Tried to split border edge in a place where it shouldn't happen!");
            }
        };

    // First two paths are a special case
    {
        std::vector<FloatingNumber> distances_to_boundary;
        std::vector<std::size_t> total_border;

        // If this patch has any "tentacles", we have to split em so that the first iteration is guaranteed to succeed
        for (auto& edge : polygon_edges)
        {
            std::copy(std::next(edge.begin()), edge.end(), std::back_inserter(total_border));
        }

        for (std::size_t i = 0; i < total_border.size(); ++i)
        {
            for (std::size_t j = 0; j < total_border.size(); ++j)
            {
                auto next = i + 1 == total_border.size() ? 0 : i + 1;
                auto prev = i == 0 ? total_border.size() - 1 : i - 1;
                if (j == next || j == prev)
                {
                    // Dont count immediate neighbors
                    continue;
                }
                if (graph.is_edge(total_border[i], total_border[j]))
                {
                    check_not_border_edge(total_border[i], total_border[j]);
                    split_edge(patch, graph, dual_graph, total_border[i], total_border[j]);
                }
            }
        }


        // Needed to prohibit our paths from hitting the boundary
        distances_to_boundary.resize(graph.vertex_count(), std::numeric_limits<FloatingNumber>::max());

        // This is O(n sqrt(n)) :(
        for (size_t i = 0; i < distances_to_boundary.size(); ++i)
        {
            auto& min = distances_to_boundary[i];
            for (auto u : total_border)
            {
                min = std::min(min, graph.get_distance(i, u));
            }
        }

        result[0] = graph.scaled_a_star(center, *midpoints[0],
            [&distances_to_boundary](size_t i) { return distances_to_boundary[i] + NUMERICAL_STABILITY_EPS; });



        // Second path

        std::size_t second_idx = 1 + (result.size() - 1) / 2;

        for (auto v : total_border)
        {
            for (auto u_it = result[0].begin(); std::next(u_it) != result[0].end(); ++u_it)
            {
                if (v == result[0].back())
                {
                    continue;
                }

                if (graph.is_edge(*u_it, v))
                {
                    check_not_border_edge(*u_it, v);
                    split_edge(patch, graph, dual_graph, *u_it, v);

                    distances_to_boundary.push_back(std::numeric_limits<FloatingNumber>::max());
                    for (auto i : total_border)
                    {
                        distances_to_boundary.back() = std::min(distances_to_boundary.back(),
                            graph.get_distance(graph.vertex_count() - 1, i));
                    }
                }
            }
        }

        std::vector<FloatingNumber> distances_to_bad_paths = distances_to_boundary;
        for (std::size_t idx = 0; idx < distances_to_bad_paths.size(); ++idx)
        {
            for (auto u : result[0])
            {
                distances_to_bad_paths[idx] = std::min(distances_to_bad_paths[idx], graph.get_distance(idx, u));
            }
        }


        result[second_idx] = graph.scaled_a_star(center, *midpoints[second_idx],
            [&distances_to_bad_paths](size_t i) { return distances_to_bad_paths[i] + NUMERICAL_STABILITY_EPS; });
    }

    for (size_t iteration = 2; iteration < result.size(); ++iteration)
    {
        // Find largest unrpocessed segment of midpoints
        decltype(result)::iterator best_begin;
        decltype(result)::iterator best_end;
        for (auto it = result.begin(); it != result.end(); )
        {
            while (it != result.cend() && !it->empty()) { ++it; }
            auto begin = it;
            while (it != result.cend() && it->empty()) { ++it; }
            auto end = it;
            if (std::distance(begin, end) > std::distance(best_begin, best_end))
            {
                best_begin = begin;
                best_end = end;
            }
        }

        auto previous_path = std::prev(best_begin);
        auto target_path = std::next(best_begin, std::distance(best_begin, best_end) / 2);
        auto next_path = best_end == result.end() ? result.begin() : best_end;

        auto previous_midpoint = midpoints[std::distance(result.begin(), previous_path)];
        auto target_midpoint = midpoints[std::distance(result.begin(), target_path)];
        auto next_midpoint = midpoints[std::distance(result.begin(), next_path)];


        std::vector<std::size_t> left{previous_path->begin() + 1, previous_path->end() - 1};
        std::vector<std::size_t> right{next_path->begin() + 1, next_path->end() - 1};
        {
            /*           .
             *        __/|\
             *      _/   | \
             *     /    /   \_
             *    /    |      \
             *  a.     |       |
             *       __|__     .b
             *         c
             */

            // We need to find ac and bc to keep away from them

            auto left_edge = polygon_edges.begin() + std::distance(result.begin(), previous_path);
            auto our_edge = polygon_edges.begin() + std::distance(result.begin(), target_path);
            auto right_edge = polygon_edges.begin() + std::distance(result.begin(), next_path);

            std::copy(previous_midpoint, std::prev(left_edge->cend()), std::back_inserter(left));
            ++left_edge;
            while (left_edge != our_edge)
            {
                std::copy(left_edge->begin(), std::prev(left_edge->end()), std::back_inserter(left));
                ++left_edge;
            }
            std::copy(our_edge->cbegin(), target_midpoint, std::back_inserter(left));


            std::copy(std::prev(std::make_reverse_iterator(next_midpoint)), std::prev(right_edge->crend()),
                std::back_inserter(right));
            if (right_edge == polygon_edges.begin())
            {
                right_edge = polygon_edges.end();
            }
            --right_edge;
            while (right_edge != our_edge)
            {
                std::copy(right_edge->rbegin(), std::prev(right_edge->rend()), std::back_inserter(right));
                --right_edge;
            }
            std::copy(right_edge->crbegin(), std::prev(std::make_reverse_iterator(target_midpoint)),
                std::back_inserter(right));
        }

        // find all pairs of edges that would prohibit us from finding a path and split em

        /* Basically get rid of this:
         *  .
         * /_\
         * |\|
         * |/|
         * |\|
         * ...
         */

        // Each path has length O(sqrt(n)) so overall it's O(n)
        for (auto u : left)
        {
            for (auto v : right)
            {
                if (graph.is_edge(u, v))
                {
                    check_not_border_edge(u, v);
                    split_edge(patch, graph, dual_graph, u, v);
                }
            }
        }


        // build the path (which is now guaranteed to exist)


        std::vector<FloatingNumber> distances_to_bad_paths;
        distances_to_bad_paths.resize(graph.vertex_count(), std::numeric_limits<FloatingNumber>::max());
        for (size_t i = 0; i < distances_to_bad_paths.size(); ++i)
        {
            auto& min = distances_to_bad_paths[i];
            for (auto u : left)
            {
                min = std::min(min, graph.get_distance(i, u));
            }
            for (auto u : right)
            {
                min = std::min(min, graph.get_distance(i, u));
            }
        }

        *target_path = graph.scaled_a_star(center, *target_midpoint,
            [&distances_to_bad_paths](size_t u) { return distances_to_bad_paths[u] + NUMERICAL_STABILITY_EPS; });
    }
    return result;
}

PolygonEdges find_polygon_edges(const ClusteringData& data,
    const SurfaceGraph& graph, size_t patch_idx)
{
    std::vector<std::vector<std::size_t>> boundary_edges;

    const auto& boundary = data.patches[patch_idx].boundary;


    for (std::size_t i = 0; i < boundary.size(); ++i)
    {
        // First point in the border is always a corner
        auto vidx = graph.index_of(boundary[i].starting_vertex);
        if (auto it = data.border_graph_vertices.find(boundary[i].starting_vertex);
            i == 0 || (it != data.border_graph_vertices.end() && it->second.size() >= 3))
        {
            if (!boundary_edges.empty())
            {
                boundary_edges.back().push_back(vidx);
            }
            boundary_edges.emplace_back();
        }
        boundary_edges.back().push_back(vidx);
    }

    boundary_edges.back().push_back(boundary_edges.front().front());


    while (boundary_edges.size() < 3)
    {
        // Bad case, we're a blob at the "edge" of the surface

        auto to_split = std::max_element(boundary_edges.begin(), boundary_edges.end(),
            [](const auto& a, const auto& b) { return a.size() < b.size(); });

        auto mid_it = median_point(graph, *to_split);

        auto new_edge = boundary_edges.emplace(std::next(to_split), mid_it, to_split->cend());
        to_split = std::prev(new_edge); // this was invalidated by the insert

        to_split->resize(std::distance(to_split->cbegin(), mid_it) + 1);
    }


    return boundary_edges;
}

template<class Iter>
void copy_side(const SurfaceGraph& graph, std::vector<HashableCoords>& side, Iter beg, Iter end)
{
    side.reserve(std::distance(beg, end));
    while (beg != end)
    {
        side.push_back(graph.coords(*beg));
        ++beg;
    }
}

std::vector<QuadPatch> quadrangulate(std::vector<ThickTriangle> triangles,
    std::size_t patch_idx, const ClusteringData& data)
{
    SurfaceGraph graph{triangles};

    DualSurfaceGraph dual_graph;
    for (size_t idx = 0; idx < triangles.size(); ++idx)
    {
        auto& tri = triangles[idx];
        dual_graph.add({to_hashable_coords(tri.a), to_hashable_coords(tri.b)}, idx);
        dual_graph.add({to_hashable_coords(tri.b), to_hashable_coords(tri.c)}, idx);
        dual_graph.add({to_hashable_coords(tri.c), to_hashable_coords(tri.a)}, idx);
    }

    auto polygon_edges = find_polygon_edges(data, graph, patch_idx);

    auto center = find_center(graph, polygon_edges);

    auto paths_to_midpoints = build_midpoint_paths(triangles, graph, dual_graph, center, polygon_edges);

    std::unordered_set<SymmetricEdge> banned;
    for (const auto& path : paths_to_midpoints)
    {
        for (std::size_t i = 1; i < path.size(); ++i)
        {
            banned.insert({graph.coords(path[i - 1]), graph.coords(path[i])});
        }
    }

    std::vector<std::size_t> starting_triangles;
    starting_triangles.reserve(polygon_edges.size());
    for (const auto& edge : polygon_edges)
    {
        starting_triangles.push_back(dual_graph.find({graph.coords(edge[0]), graph.coords(edge[1])}).first);
    }
    // hack: starting edge corresponds to the previous patch
    std::rotate(starting_triangles.begin(), std::next(starting_triangles.begin()), starting_triangles.end());

    auto colors = dual_graph.paint_quads(triangles, banned);

    std::vector<QuadPatch> result{starting_triangles.size()};

    {
        for (std::size_t i = 0; i < triangles.size(); ++i)
        {
            if (colors[i] == DualSurfaceGraph::COLOR_NONE)
            {
                continue;
            }
            result[colors[i]].triangles.push_back(triangles[i]);
        }
    }

    for (std::size_t i = 0; i < starting_triangles.size(); ++i)
    {
        std::size_t next = i + 1 == starting_triangles.size() ? 0 : i + 1;

        // from center to midpoint 1
        copy_side(graph, result[i].boundary[0], paths_to_midpoints[i].begin(), paths_to_midpoints[i].end());

        // from midpoint 1 to corner
        auto midpoint1_it = std::find(polygon_edges[i].begin(), polygon_edges[i].end(),
            paths_to_midpoints[i].back());
        copy_side(graph, result[i].boundary[1], midpoint1_it, polygon_edges[i].end());

        // from corner to midpoint 2
        auto midpoint2_it = std::find(polygon_edges[next].begin(), polygon_edges[next].end(),
            paths_to_midpoints[next].back());
        copy_side(graph, result[i].boundary[2], polygon_edges[next].begin(), std::next(midpoint2_it));

        // from corner to center
        copy_side(graph, result[i].boundary[3], paths_to_midpoints[next].rbegin(), paths_to_midpoints[next].rend());
    }

    return result;
}
