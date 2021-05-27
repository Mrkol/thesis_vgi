#pragma once

#include "DataTypes.hpp"

#include <unordered_map>
#include <unordered_set>


class DualSurfaceGraph
{
public:
	static constexpr std::size_t INVALID = std::numeric_limits<std::size_t>::max();

	DualSurfaceGraph() = default;

	explicit DualSurfaceGraph(const std::vector<ThickTriangle>& triangles)
    {
	    for (std::size_t i = 0; i < triangles.size(); ++i)
        {
	        for (auto& e : triangle_edges(triangles[i]))
            {
                add(e, i);
            }
        }
    }

    [[nodiscard]] std::pair<std::size_t, std::size_t> find(const SymmetricEdge& edge) const
	{
		auto it = impl.find(edge);

		if (it == impl.end())
		{
			throw std::logic_error("Edge not in hash map!");
		}

		return it->second;
	}

	[[nodiscard]] std::size_t find_not(const SymmetricEdge& edge, std::size_t idx) const
	{
		auto[first, second] = find(edge);
		return first != idx ? first : second;
	}

	void add(SymmetricEdge edge, std::size_t idx)
	{
	    auto it = impl.find(edge);

	    if (it == impl.end())
        {
	        impl.emplace(edge, std::pair{idx, INVALID});
	        return;
        }

        if (it->second.first != idx && it->second.second == INVALID)
        {
            it->second.second = idx;
            return;
        }
        
        throw std::logic_error("Trying to hash a non-manifold surface!");
	}

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
     *
     * Returns (a, m, b) where m is the new point
     */
    std::tuple<HashableCoords, HashableCoords, std::optional<HashableCoords>>
        split_edge(std::vector<ThickTriangle>& patch, HashableCoords u, HashableCoords v);

    void remove_triangle(HashableCoords a, HashableCoords b, HashableCoords c);

    static constexpr std::size_t COLOR_NONE = std::numeric_limits<std::size_t>::max();
    [[nodiscard]] std::vector<std::size_t> paint_quads(
        const std::vector<ThickTriangle>& patch, const std::unordered_set<SymmetricEdge>& banned) const;

    [[nodiscard]] auto begin() const { return impl.begin(); }
    [[nodiscard]] auto end() const { return impl.begin(); }

private:
    void remove(SymmetricEdge edge, std::size_t from_idx);

private:
	std::unordered_map<SymmetricEdge, std::pair<std::size_t, std::size_t>> impl;
};
