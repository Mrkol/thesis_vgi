#include "Quadrangulate.hpp"

#include <vector>
#include <fstream>
#include <unordered_map>
#include <queue>

#include "../DataTypes.hpp"
#include "Clustering.hpp"
#include "../SurfaceHashTable.hpp"


// TODO: This is not a good abstraction, redesign
class SurfaceGraph
{
public:
    explicit SurfaceGraph(const std::vector<ThickTriangle>& triangles)
    {
        // TODO: LESS ALLOCATIONS FFS
        for (const auto& triangle : triangles)
        {
            ThickVertex ThickTriangle::*fields[] = {&ThickTriangle::a, &ThickTriangle::b, &ThickTriangle::c};
            for (auto field : fields)
            {
                auto vertex = to_hashable_coords(triangle.*field);
                auto it = vertex_indices.find(vertex);
                if (it == vertex_indices.end())
                {
                    vertex_indices.insert(it, {vertex, vertex_indices.size()});
                }
            }
        }

        vertices.resize(vertex_indices.size());
        for (auto&[coords, idx] : vertex_indices)
        {
            vertices[idx] = coords;
        }

        adjacency_list.resize(vertex_indices.size());
        for (auto& triangle : triangles)
        {
            ThickVertex ThickTriangle::*fields[] = {&ThickTriangle::a, &ThickTriangle::b, &ThickTriangle::c};
            for (auto first : fields)
            {
                for (auto second : fields)
                {
                    if (first != second)
                    {
                        auto first_coords = to_hashable_coords(triangle.*first);
                        auto second_coords = to_hashable_coords(triangle.*second);

                        adjacency_list[vertex_indices[first_coords]].insert(vertex_indices[second_coords]);

                        SymmetricPair<std::size_t> edge{vertex_indices[first_coords], vertex_indices[second_coords]};
                        lengths[edge] = length({first_coords, second_coords});
                    }
                }
            }
        }
    }

    [[nodiscard]] const std::unordered_set<std::size_t>& adjacent_to(std::size_t idx) const
    {
        return adjacency_list[idx];
    }

    [[nodiscard]] FloatingNumber get_length(std::size_t u, std::size_t v) const
    {
        auto it = lengths.find({u, v});
        if (it == lengths.end())
        {
            throw std::logic_error("Length missing!");
        }
        return it->second;
    }

    [[nodiscard]] FloatingNumber get_distance(std::size_t u, std::size_t v) const
    {
        return length({vertices[u], vertices[v]});
    }

    [[nodiscard]] std::size_t index_of(HashableCoords point) const
    {
        auto it = vertex_indices.find(point);
        if (it == vertex_indices.end())
        {
            throw std::logic_error("This vertex is not indexed in this graph!");
        }
        return it->second;
    }

    [[nodiscard]] HashableCoords coords(std::size_t idx) const
    {
        return vertices[idx];
    }

    [[nodiscard]] bool is_edge(std::size_t u, std::size_t v)
    {
        return lengths.contains({u, v});
    }

    [[nodiscard]] std::size_t vertex_count() const
    {
        return vertex_indices.size();
    }

    // Look at the picture where this thing is used
    void split_edge(HashableCoords u, HashableCoords v, HashableCoords a, HashableCoords b, HashableCoords m)
    {
        auto u_idx = index_of(u);
        auto v_idx = index_of(v);
        auto a_idx = index_of(a);
        auto b_idx = index_of(b);

        std::size_t m_idx = vertices.size();
        vertex_indices[m] = m_idx;
        vertices.push_back(m);
        adjacency_list.push_back({a_idx, b_idx, u_idx, v_idx});

        adjacency_list[u_idx].erase(v_idx);
        adjacency_list[v_idx].erase(u_idx);

        adjacency_list[a_idx].insert(m_idx);
        adjacency_list[b_idx].insert(m_idx);
        adjacency_list[u_idx].insert(m_idx);
        adjacency_list[v_idx].insert(m_idx);

        lengths.erase(SymmetricPair<std::size_t>{u_idx, v_idx});

        lengths[SymmetricPair<std::size_t>{a_idx, m_idx}] = length({a, m});
        lengths[SymmetricPair<std::size_t>{v_idx, m_idx}] = length({v, m});
        lengths[SymmetricPair<std::size_t>{b_idx, m_idx}] = length({b, m});
        lengths[SymmetricPair<std::size_t>{u_idx, m_idx}] = length({u, m});
    }

private:
    std::unordered_map<HashableCoords, std::size_t> vertex_indices;
    std::vector<HashableCoords> vertices;

    std::vector<std::unordered_set<std::size_t>> adjacency_list;
    std::unordered_map<SymmetricPair<std::size_t>, FloatingNumber> lengths;
};

// TODO: this can be made faster
std::vector<FloatingNumber> dijkstra(const SurfaceGraph& graph,  std::size_t start,
    const std::function<FloatingNumber(std::size_t)>& scale_factor = [](std::size_t) { return 1; },
    const std::function<FloatingNumber(std::size_t)>& heuristic = [](std::size_t) { return 0; })
{
    std::vector<FloatingNumber> distances;
    distances.resize(graph.vertex_count(), std::numeric_limits<FloatingNumber>::max());
    distances[start] = 0;

    using QueueElement = std::pair<FloatingNumber, std::size_t>;
    std::priority_queue<QueueElement, std::vector<QueueElement>, std::greater<>> queue;
    queue.emplace(0, start);


    while (!queue.empty())
    {
        auto current = queue.top().second;
        queue.pop();

        for (auto adjacent : graph.adjacent_to(current))
        {
            auto relaxed_distance = distances[current]
                + graph.get_length(current, adjacent) / scale_factor(adjacent);
            if (relaxed_distance < distances[adjacent])
            {
                distances[adjacent] = relaxed_distance;
                queue.emplace(relaxed_distance + heuristic(adjacent), adjacent);
            }
        }
    }

    return distances;
}

std::vector<std::size_t> recontstruct_shortest_path(
    const SurfaceGraph& graph, const std::vector<FloatingNumber>& distances,
    std::size_t start, std::size_t end)
{
    if (distances[end] == std::numeric_limits<FloatingNumber>::max())
    {
        throw std::logic_error("Path could not be built: vertex is unreachable.");
    }

    std::vector<std::size_t> result{end};
    while (result.back() != start)
    {
        auto& adjacents = graph.adjacent_to(result.back());
        auto comparator = [&distances](std::size_t i, std::size_t j) { return distances[i] < distances[j]; };
        result.push_back(*std::min_element(adjacents.begin(), adjacents.end(), comparator));
    }

    std::reverse(result.begin(), result.end());
    return result;
}

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
        total_length += graph.get_length(path[i], path[i - 1]);
    }

    auto it = path.begin();
    FloatingNumber current = 0;
    while (std::next(it) != path.end())
    {
        auto potential = current + graph.get_length(*it, *std::next(it));
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

    std::size_t m = 0;

    {
        auto distances = dijkstra(graph, corners[0]);
        m = *median_point(graph, recontstruct_shortest_path(graph, distances, corners[0], corners[1]));
    }

    auto distances = dijkstra(graph, m);

    std::vector<std::size_t> result;
    result.reserve(corners.size());

    for (std::size_t i = 2; i < corners.size(); ++i)
    {
        result.push_back(*median_point(graph, recontstruct_shortest_path(graph, distances, m, corners[i])));
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

    std::size_t naive_candidate = 0;
    if (current.size() == 2)
    {
        auto distances = dijkstra(graph, current[0]);
        naive_candidate = *median_point(graph, recontstruct_shortest_path(graph, distances, current[0], current[1]));
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

void split_edge(std::vector<ThickTriangle>& patch, SurfaceGraph& graph, SurfaceHashTable<FloatingNumber>& dual_graph,
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

    auto a = to_hashable_coords(get_third(patch[first_idx], {u, v}));
    auto b = to_hashable_coords(get_third(patch[second_idx], {u, v}));


    // Split actual geometric data
    ThickVertex m_thick;
    size_t first_prime_idx = 0;
    size_t second_prime_idx = 0;
    {
        m_thick = midpoint(get_third(patch[second_idx], {b, v}), get_third(patch[first_idx], {a, u}));

        // Duplicate first and shift u -> m
        first_prime_idx = patch.size();
        patch.push_back(patch[first_idx]);
        get_third(patch.back(), {a, v}) = m_thick;

        // Duplicate second and shift v -> m
        second_prime_idx = patch.size();
        patch.push_back(patch[second_idx]);
        get_third(patch.back(), {b, u}) = m_thick;

        // For first, shift v -> m
        get_third(patch[first_idx], {a, u}) = m_thick;

        // for second, shift u -> m
        get_third(patch[second_idx], {b, v}) = m_thick;
    }

    auto m = to_hashable_coords(m_thick);

    // Split hash table data
    // Don't try to read this. Look at the picture above.
    {
        dual_graph.remove_triangle(a, u, v);
        dual_graph.remove_triangle(u, v, b);
        dual_graph.add({a, v}, first_prime_idx);
        dual_graph.add({v, b}, second_idx);
        dual_graph.add({b, u}, second_prime_idx);
        dual_graph.add({u, a}, first_idx);

        dual_graph.add({a, m}, first_idx);
        dual_graph.add({a, m}, first_prime_idx);
        dual_graph.add({v, m}, first_prime_idx);
        dual_graph.add({v, m}, second_idx);
        dual_graph.add({b, m}, second_idx);
        dual_graph.add({b, m}, second_prime_idx);
        dual_graph.add({u, m}, second_prime_idx);
        dual_graph.add({u, m}, first_idx);
    }

    // Split graph
    graph.split_edge(u, v, a, b, m);
}

std::vector<std::vector<std::size_t>> build_midpoint_paths(
    std::vector<ThickTriangle>& patch, SurfaceGraph& graph, SurfaceHashTable<FloatingNumber>& dual_graph,
    size_t center, const PolygonEdges& polygon_edges)
{
    // This hardcoded epsilon might be a bad idea
    static constexpr FloatingNumber NUMERICAL_STABILITY_EPS = 1e-6;

    std::vector<PolygonEdges::value_type::const_iterator> midpoints;
    midpoints.reserve(polygon_edges.size());
    for (const auto& edge : polygon_edges)
    {
        midpoints.push_back(median_point(graph, edge));
    }

    std::vector<std::vector<size_t>> result{midpoints.size()};

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

        {
            auto distances = dijkstra(graph, center,
                                      [&distances_to_boundary](size_t i) { return distances_to_boundary[i] + NUMERICAL_STABILITY_EPS; },
                                      [m = *midpoints[0], &graph](size_t v) { return graph.get_distance(m, v); });

            result[0] = recontstruct_shortest_path(graph, distances, center, *midpoints[0]);
        }



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


        auto distances = dijkstra(graph, center,
            [&distances_to_bad_paths](size_t i) { return distances_to_bad_paths[i] + NUMERICAL_STABILITY_EPS; },
            [m = *midpoints[second_idx], &graph](size_t v) { return graph.get_distance(m, v); });

        result[second_idx] = recontstruct_shortest_path(graph, distances, center, *midpoints[second_idx]);
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

            std::copy(previous_midpoint, std::prev(left_edge->end()), std::back_inserter(left));
            ++left_edge;
            while (left_edge != our_edge)
            {
                std::copy(left_edge->begin(), std::prev(left_edge->end()), std::back_inserter(left));
                ++left_edge;
            }
            std::copy(our_edge->begin(), target_midpoint, std::back_inserter(left));


            std::copy(std::prev(std::make_reverse_iterator(next_midpoint)), std::prev(right_edge->rend()),
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
            std::copy(right_edge->rbegin(), std::prev(std::make_reverse_iterator(target_midpoint)),
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

        auto distances = dijkstra(graph, center,
            [&distances_to_bad_paths](size_t u) { return distances_to_bad_paths[u] + NUMERICAL_STABILITY_EPS; },
            [&graph, m = *target_midpoint](size_t u) { return graph.get_distance(u, m); });

        *target_path = recontstruct_shortest_path(graph, distances, center, *target_midpoint);
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

constexpr std::size_t COLOR_NONE = std::numeric_limits<std::size_t>::max();

std::vector<std::size_t> paint_quads(
    const std::vector<ThickTriangle>& patch, const SurfaceHashTable<FloatingNumber>& dual_graph,
    const std::vector<std::size_t>& starting_triangles, const std::unordered_set<SymmetricEdge>& banned)
{
    std::vector<std::size_t> triangle_colors;
    triangle_colors.resize(patch.size(), COLOR_NONE);

    for (std::size_t i = 0; i < starting_triangles.size(); ++i)
    {
        std::stack<std::size_t> stack;
        stack.push(starting_triangles[i]);

        while (!stack.empty())
        {
            std::size_t idx = stack.top();
            stack.pop();

            if (triangle_colors[idx] != COLOR_NONE)
            {
                continue;
            }

            triangle_colors[idx] = i;

            auto edges = triangle_edges(patch[idx]);
            for (auto& e : edges)
            {
                auto next = dual_graph.find_not(e, idx);
                if (!banned.contains(e) && next != SurfaceHashTable<FloatingNumber>::INVALID
                    && triangle_colors[next] == COLOR_NONE)
                {
                    stack.push(next);
                }
            }
        }
    }

    return triangle_colors;
}

void quadrangulate(const std::filesystem::path& patchfile, std::size_t patch_idx, const ClusteringData& data)
{
    std::vector<ThickTriangle> patch;
    patch.reserve(file_size(patchfile) / sizeof(ThickTriangle));
    {
        std::ifstream input{patchfile, std::ios_base::binary};
        ThickTriangle current;
        while (input.read(reinterpret_cast<char*>(&current), sizeof(current)))
        {
            patch.push_back(current);
        }
    }

    SurfaceGraph graph{patch};

    SurfaceHashTable<FloatingNumber> dual_graph;
    for (size_t idx = 0; idx < patch.size(); ++idx)
    {
        dual_graph.add({to_hashable_coords(patch[idx].a), to_hashable_coords(patch[idx].b)}, idx);
        dual_graph.add({to_hashable_coords(patch[idx].b), to_hashable_coords(patch[idx].c)}, idx);
        dual_graph.add({to_hashable_coords(patch[idx].c), to_hashable_coords(patch[idx].a)}, idx);
    }

    auto polygon_edges = find_polygon_edges(data, graph, patch_idx);

    auto center = find_center(graph, polygon_edges);

    auto paths_to_midpoints = build_midpoint_paths(patch, graph, dual_graph, center, polygon_edges);

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

    auto colors = paint_quads(patch, dual_graph, starting_triangles, banned);

    remove(patchfile);

    std::vector<std::ofstream> quads;
    quads.reserve(starting_triangles.size());
    for (std::size_t i = 0; i < starting_triangles.size(); ++i)
    {
        quads.emplace_back(patchfile.parent_path() / (patchfile.filename().string() + ":" + std::to_string(i)),
            std::ios_base::binary);
    }


    for (std::size_t i = 0; i < patch.size(); ++i)
    {
        if (colors[i] == COLOR_NONE)
        {
            continue;
        }
        quads[colors[i]].write(reinterpret_cast<char*>(&(patch[i])), sizeof(patch[i]));
    }
}


