#include "SurfaceGraph.hpp"

#include <queue>


SurfaceGraph::SurfaceGraph(const std::vector<ThickTriangle>& triangles)
{
    // TODO: LESS ALLOCATIONS FFS
    for (const auto& triangle : triangles)
    {
        ThickVertex ThickTriangle::*fields[] = {&ThickTriangle::a, &ThickTriangle::b, &ThickTriangle::c};
        for (auto field : fields)
        {
            auto vertex = to_hashable_coords(triangle.*field);
            if (!vertex_indices.contains(vertex))
            {
                vertex_indices.emplace(vertex, vertex_indices.size());
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
                }
            }
        }
    }
}

const std::unordered_set<size_t>& SurfaceGraph::adjacent_to(size_t idx) const
{
    return adjacency_list[idx];
}

FloatingNumber SurfaceGraph::get_distance(size_t u, size_t v) const
{
    return length({vertices[u], vertices[v]});
}

size_t SurfaceGraph::index_of(HashableCoords point) const
{
    auto it = vertex_indices.find(point);
    if (it == vertex_indices.end())
    {
        throw std::logic_error("This vertex is not indexed in this graph!");
    }
    return it->second;
}

HashableCoords SurfaceGraph::coords(size_t idx) const
{
    return vertices[idx];
}

bool SurfaceGraph::is_edge(size_t u, size_t v)
{
    return u < vertices.size() && adjacency_list[u].contains(v);
}

size_t SurfaceGraph::vertex_count() const
{
    return vertex_indices.size();
}

// Look at the picture where this thing is used
void SurfaceGraph::split_edge(HashableCoords u, HashableCoords v,
    HashableCoords a, std::optional<HashableCoords> b, HashableCoords m)
{
    auto u_idx = index_of(u);
    auto v_idx = index_of(v);
    auto a_idx = index_of(a);
    std::size_t b_idx{};
    if (b)
    {
        b_idx = index_of(b.value());
    }

    size_t m_idx = vertices.size();
    vertex_indices[m] = m_idx;
    vertices.push_back(m);
    adjacency_list.push_back({a_idx, u_idx, v_idx});
    if (b)
    {
        adjacency_list.back().insert(b_idx);
    }

    adjacency_list[u_idx].erase(v_idx);
    adjacency_list[v_idx].erase(u_idx);

    adjacency_list[a_idx].insert(m_idx);
    if (b)
    {
        adjacency_list[b_idx].insert(m_idx);
    }
    adjacency_list[u_idx].insert(m_idx);
    adjacency_list[v_idx].insert(m_idx);
}

// TODO: this can be made faster
template<bool ASTAR, bool SCALED>
std::vector<FloatingNumber> generic_pathfinder(const SurfaceGraph& graph,  std::size_t start,
    fu2::unique_function<FloatingNumber(std::size_t)> scale_factor = {},
    std::size_t target = 0)
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

        if constexpr (ASTAR)
        {
            if (current == target)
            {
                return distances;
            }
        }

        for (auto adjacent : graph.adjacent_to(current))
        {
            auto edge_length = graph.get_distance(current, adjacent);
            if constexpr (SCALED)
            {
                edge_length /= scale_factor(adjacent);
            }
            auto relaxed_distance = distances[current] + edge_length;

            if (relaxed_distance < distances[adjacent])
            {
                distances[adjacent] = relaxed_distance;
                if constexpr (ASTAR)
                {
                    relaxed_distance += graph.get_distance(adjacent, target);
                }
                queue.emplace(relaxed_distance, adjacent);
            }
        }
    }

    return distances;
}

std::vector<FloatingNumber> SurfaceGraph::dijkstra(std::size_t start) const
{
    return generic_pathfinder<false, false>(*this, start);
}

std::vector<std::size_t> SurfaceGraph::recontstruct_shortest_path(
    const std::vector<FloatingNumber>& distances,
    std::size_t start, std::size_t end) const
{
    if (distances[end] == std::numeric_limits<FloatingNumber>::max())
    {
        throw std::logic_error("Path could not be built: vertex is unreachable.");
    }

    std::vector<std::size_t> result{end};
    while (result.back() != start)
    {
        auto& adjacents = adjacent_to(result.back());
        auto comparator = [&distances](std::size_t i, std::size_t j) { return distances[i] < distances[j]; };
        result.push_back(*std::min_element(adjacents.begin(), adjacents.end(), comparator));
    }

    std::reverse(result.begin(), result.end());
    return result;
}

std::vector<std::size_t> SurfaceGraph::a_star(std::size_t start, std::size_t finish) const
{
    auto distances = generic_pathfinder<true, false>(*this, start, {}, finish);
    return recontstruct_shortest_path(distances, start, finish);
}

std::vector<std::size_t> SurfaceGraph::scaled_a_star(std::size_t start, std::size_t finish,
    fu2::unique_function<FloatingNumber(std::size_t)> scale_factor) const
{
    auto distances = generic_pathfinder<true, true>(*this, start, std::move(scale_factor), finish);
    return recontstruct_shortest_path(distances, start, finish);
}
