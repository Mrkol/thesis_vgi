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

    void split_edge(std::size_t u, std::size_t v, HashableCoords m)
    {
        if (u > vertex_count() || v > vertex_count() || !lengths.contains({u, v}))
        {
            throw std::logic_error("Edge to be splitted not part of the graph!");
        }

        std::size_t idx = vertices.size();
        vertex_indices[m] = idx;
        vertices.push_back(m);
        adjacency_list.emplace_back();

        adjacency_list[u].erase(v);
        adjacency_list[v].erase(u);

        auto it1 = std::find_if(adjacency_list[u].begin(), adjacency_list[u].end(),
            [this, v](std::size_t i) { return adjacency_list[v].contains(i); });
        auto it2 = std::find_if(it1, adjacency_list[u].end(),
                                [this, v](std::size_t i) { return adjacency_list[v].contains(i); });

        if (it1 == adjacency_list[u].end() || it2 == adjacency_list[u].end())
        {
            throw std::logic_error("Trying to split an edge on the border! This shouldn't happen!");
        }

        auto a = *it1;
        auto b = *it2;

        adjacency_list[a].insert(idx);
        adjacency_list[b].insert(idx);
        adjacency_list[u].insert(idx);
        adjacency_list[v].insert(idx);

        lengths.erase(SymmetricPair<std::size_t>{u, v});

        lengths[SymmetricPair<std::size_t>{a, idx}] = length({vertices[a], m});
        lengths[SymmetricPair<std::size_t>{v, idx}] = length({vertices[v], m});
        lengths[SymmetricPair<std::size_t>{b, idx}] = length({vertices[b], m});
        lengths[SymmetricPair<std::size_t>{u, idx}] = length({vertices[u], m});

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

    if (path.size() == 1)
    {
        return path.begin();
    }

    std::vector<FloatingNumber> prefix_sums;
    prefix_sums.reserve(path.size() + 1);
    prefix_sums.push_back(0);

    for (std::size_t i = 1; i < path.size(); ++i)
    {
        prefix_sums.push_back(prefix_sums.back() + graph.get_length(path[i], path[i - 1]));
    }

    return path.begin() + std::distance(
        prefix_sums.begin(),
        std::lower_bound(prefix_sums.begin(), prefix_sums.end(), prefix_sums.back() / 2));
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

std::size_t find_center(const SurfaceGraph& graph, const std::vector<std::size_t>& corners)
{
    std::vector<std::size_t> current = corners;
    while (current.size() > 2)
    {
        current = find_center_recurse(graph, current);
    }

    std::size_t candidate = 0;
    if (current.size() == 2)
    {
        auto distances = dijkstra(graph, current[0]);
        candidate = *median_point(graph, recontstruct_shortest_path(graph, distances, current[0], current[1]));
    }
    else
    {
        candidate = current[0];
    }

    // TODO: DFS proper with max depth = 2-3
    for (auto adjacent : graph.adjacent_to(candidate))
    {
        if (graph.adjacent_to(candidate).size() < graph.adjacent_to(adjacent).size())
        {
            candidate = adjacent;
        }
    }

    return candidate;
}

std::vector<std::vector<std::size_t>> build_midpoint_paths(
    std::vector<ThickTriangle>& patch, SurfaceGraph& graph, SurfaceHashTable<FloatingNumber>& dual_graph,
    size_t center, const std::vector<std::size_t>& midpoints)
{
    std::vector<std::vector<size_t>> result{midpoints.size()};

    {
        auto distances = dijkstra(graph, center, [](size_t) { return 1; },
            [m = midpoints[0], &graph](size_t v) { return graph.get_distance(m, v); });
        result[0] = recontstruct_shortest_path(graph, distances, center, midpoints[0]);
    }

    for (size_t iteration = 1; iteration < result.size(); ++iteration)
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


        auto target_midpoint = midpoints[std::distance(result.begin(), target_path)];

        // find all pairs of edges that would prohibit us from finding a path and split em

        /* Basically get rid of this:
         *  .
         * /_\
         * |\|
         * |/|
         * |\|
         * ...
         */

        // On the first iteration, prev = next, so the picture above is impossible
        if (iteration > 1)
        {
            std::vector<SymmetricPair<size_t>> edges;
            // Each path has length O(sqrt(n)) so overall it's O(n)
            for (size_t i = 1; i < previous_path->size(); ++i)
            {
                for (size_t j = 1; j < next_path->size(); ++j)
                {
                    auto u = (*previous_path)[i];
                    auto v = (*next_path)[j];
                    if (graph.is_edge(u, v))
                    {
                        edges.push_back({u, v});
                    }
                }
            }


            for (auto[u_idx, v_idx] : edges)
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
                graph.split_edge(graph.index_of(u), graph.index_of(v), m);
            }
        }


        // build the path (which is now guaranteed to exist)

        std::vector<FloatingNumber> distances_to_bad_paths;
        distances_to_bad_paths.resize(graph.vertex_count());
        for (size_t i = 0; i < distances_to_bad_paths.size(); ++i)
        {
            auto& min = distances_to_bad_paths[i];
            min = std::numeric_limits<FloatingNumber>::max();
            for (auto u : *previous_path)
            {
                min = std::min(min, graph.get_distance(i, u));
            }
            for (auto u : *next_path)
            {
                min = std::min(min, graph.get_distance(i, u));
            }
        }

        auto distances = dijkstra(graph, center,
            [&distances_to_bad_paths](size_t u) { return distances_to_bad_paths[u]; },
            [&graph, m = target_midpoint](size_t u) { return graph.get_distance(u, m); });

        *target_path = recontstruct_shortest_path(graph, distances, center, target_midpoint);
    }
    return result;
}

struct PolygonInfo
{
    std::vector<std::size_t> corners;
    std::vector<std::size_t> midpoints;
    // vertices guaranteed to be on the "next" edge immediately after the corners
    std::vector<std::size_t> corner_neighbors;
};

PolygonInfo find_polygon_info(const ClusteringData& data,
    const SurfaceGraph& graph, size_t patch_idx)
{
    std::vector<std::size_t> corners;
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
            corners.push_back(vidx);
            boundary_edges.emplace_back();
        }
        boundary_edges.back().push_back(vidx);
    }

    boundary_edges.back().push_back(corners.front());


    while (corners.size() < 3)
    {
        // Bad case, we're a blob at the "edge" of the surface

        auto to_split = std::max_element(boundary_edges.begin(), boundary_edges.end(),
            [](const auto& a, const auto& b) { return a.size() < b.size(); });

        auto mid_it = median_point(graph, *to_split);

        corners.insert(std::next(corners.begin(), std::distance(boundary_edges.begin(), to_split) + 1), *mid_it);

        auto new_edge = boundary_edges.emplace(std::next(to_split), mid_it, to_split->cend());
        to_split = std::prev(new_edge); // this was invalidated by the insert

        to_split->resize(std::distance(to_split->cbegin(), mid_it) + 1);
    }

    std::vector<std::size_t> midpoints;
    midpoints.reserve(boundary_edges.size());
    for (const auto& edge : boundary_edges)
    {
        midpoints.push_back(*median_point(graph, edge));
    }

    std::vector<std::size_t> corner_neighbors;
    corner_neighbors.reserve(boundary_edges.size());
    for (const auto& edge : boundary_edges)
    {
        corner_neighbors.push_back(edge[1]);
    }

    return {std::move(corners), std::move(midpoints), std::move(corner_neighbors)};
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
    remove(patchfile);

    SurfaceGraph graph{patch};

    SurfaceHashTable<FloatingNumber> dual_graph;
    for (size_t idx = 0; idx < patch.size(); ++idx)
    {
        dual_graph.add({to_hashable_coords(patch[idx].a), to_hashable_coords(patch[idx].b)}, idx);
        dual_graph.add({to_hashable_coords(patch[idx].b), to_hashable_coords(patch[idx].c)}, idx);
        dual_graph.add({to_hashable_coords(patch[idx].c), to_hashable_coords(patch[idx].a)}, idx);
    }

    auto[corners, midpoints, corner_neighbors] = find_polygon_info(data, graph, patch_idx);

    auto center = find_center(graph, corners);

    auto paths_to_midpoints = build_midpoint_paths(patch, graph, dual_graph, center, midpoints);

    std::unordered_set<SymmetricEdge> banned;
    for (const auto& path : paths_to_midpoints)
    {
        for (std::size_t i = 1; i < path.size(); ++i)
        {
            banned.insert({graph.coords(path[i - 1]), graph.coords(path[i])});
        }
    }

    std::vector<std::size_t> starting_triangles;
    starting_triangles.reserve(corners.size());
    for (std::size_t i = 0; i < corners.size(); ++i)
    {
        starting_triangles.push_back(dual_graph.find(
            {graph.coords(corners[i]), graph.coords(corner_neighbors[i])}).first);
    }

    auto colors = paint_quads(patch, dual_graph, starting_triangles, banned);

    std::vector<std::ofstream> quads;
    for (std::size_t i = 0; i < midpoints.size(); ++i)
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


