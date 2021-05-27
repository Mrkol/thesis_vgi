#include "DualSurfaceGraph.hpp"

#include <stack>


std::vector<std::size_t> DualSurfaceGraph::paint_quads(
    const std::vector<ThickTriangle>& patch, const std::unordered_set<SymmetricEdge>& banned) const
{
    std::vector<std::size_t> triangle_colors;
    triangle_colors.resize(patch.size(), COLOR_NONE);

    std::size_t current_color = 0;
    for (std::size_t i = 0; i < patch.size(); ++i)
    {
        if (triangle_colors[i] != COLOR_NONE)
        {
            continue;
        }

        std::stack<std::size_t> stack;
        stack.push(i);

        while (!stack.empty())
        {
            std::size_t idx = stack.top();
            stack.pop();

            if (triangle_colors[idx] != COLOR_NONE)
            {
                continue;
            }

            triangle_colors[idx] = current_color;

            auto edges = triangle_edges(patch[idx]);
            for (auto& e : edges)
            {
                auto next = find_not(e, idx);
                if (!banned.contains(e) && next != DualSurfaceGraph::INVALID
                    && triangle_colors[next] == COLOR_NONE)
                {
                    stack.push(next);
                }
            }
        }

        ++current_color;
    }

    return triangle_colors;
}

std::tuple<HashableCoords, HashableCoords, std::optional<HashableCoords>>
    DualSurfaceGraph::split_edge(std::vector<ThickTriangle>& patch, HashableCoords u, HashableCoords v)
{
    auto[first_idx, second_idx] = find({u, v});

    if (first_idx == INVALID)
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
    HashableCoords b = {};
    if (second_idx != INVALID)
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

        if (second_idx != INVALID)
        {
            // Duplicate second and shift v -> m
            second_prime_idx = patch.size();
            patch.push_back(patch[second_idx]);
            get_third(patch.back(), {b, u}) = m_thick;
        }

        // For first, shift v -> m
        get_third(patch[first_idx], {a, u}) = m_thick;

        if (second_idx != INVALID)
        {
            // for second, shift u -> m
            get_third(patch[second_idx], {b, v}) = m_thick;
        }
    }

    HashableCoords m = to_hashable_coords(m_thick);

    // Split hash table data
    // Don't try to read this. Look at the picture above.
    {
        remove_triangle(a, u, v);
        if (second_idx != INVALID)
        {
            remove_triangle(u, v, b);
        }

        add({a, v}, first_prime_idx);
        if (second_idx != INVALID)
        {
            add({v, b}, second_idx);
            add({b, u}, second_prime_idx);
        }
        add({u, a}, first_idx);

        add({a, m}, first_idx);
        add({a, m}, first_prime_idx);
        add({v, m}, first_prime_idx);
        if (second_idx != INVALID)
        {
            add({v, m}, second_idx);
            add({b, m}, second_idx);
            add({b, m}, second_prime_idx);
            add({u, m}, second_prime_idx);
        }
        add({u, m}, first_idx);
    }

    return {a, m, second_idx != INVALID ? std::make_optional(b) : std::nullopt};
}

void DualSurfaceGraph::remove(SymmetricEdge edge, std::size_t from_idx)
{
    auto it = impl.find(edge);

    if (it == impl.end())
    {
        throw std::logic_error("Edge is not in this table!");
    }

    if (it->second.first == from_idx)
    {
        if (it->second.second == INVALID)
        {
            impl.erase(it);
        }
        else
        {
            std::swap(it->second.first, it->second.second);
            it->second.second = INVALID;
        }
        return;
    }

    if (it->second.second == from_idx)
    {
        it->second.second = INVALID;
        return;
    }

    throw std::logic_error("Trying to replace an edge!");
}

/**
 *        a
 *       /\
 *      /  \
 *     /    \
 *    /______\
 *   b        c
 */
void DualSurfaceGraph::remove_triangle(HashableCoords a, HashableCoords b, HashableCoords c)
{
    //correct due to the mesh being a manifold
    auto [ab1, ab2] = find({a, b});
    auto [bc1, bc2] = find({b, c});

    std::size_t idx = 0;
    if ((ab1 == bc1 || ab1 == bc2) && ab1 != INVALID) { idx = ab1; }
    else if ((ab2 == bc1 || ab2 == bc2) && ab2 != INVALID) { idx = ab2; }
    else { throw std::logic_error{"Something went very wrong!"}; }

    remove({a, b}, idx);
    remove({b, c}, idx);
    remove({c, a}, idx);
}
