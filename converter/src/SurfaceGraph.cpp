#include "SurfaceGraph.hpp"


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
