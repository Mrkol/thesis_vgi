#pragma once

#include "ToPlainConverters.hpp"
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

	std::pair<std::size_t, std::size_t> find(const SymmetricEdge& edge) const
	{
		auto it = impl.find(edge);

		if (it == impl.end())
		{
			throw std::logic_error("Edge not in hash map!");
		}

		return it->second;
	}

	std::size_t find_not(const SymmetricEdge& edge, std::size_t idx) const
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

	/**
	 *        a
	 *       /\
	 *      /  \
	 *     /    \
	 *    /______\
	 *   b        c
	 */
	void remove_triangle(HashableCoords a, HashableCoords b, HashableCoords c)
    {
	    //correct due to the mesh being a manifold
        auto[ab1, ab2] = find({a, b});
        auto[bc1, bc2] = find({b, c});

        std::size_t idx = 0;
        if ((ab1 == bc1 || ab1 == bc2) && ab1 != INVALID) { idx = ab1; }
        else if ((ab2 == bc1 || ab2 == bc2) && ab2 != INVALID) { idx = ab2; }
        else { throw std::logic_error{"Something went very wrong!"}; }

        remove({a, b}, idx);
        remove({b, c}, idx);
        remove({c, a}, idx);
    }

    static constexpr std::size_t COLOR_NONE = std::numeric_limits<std::size_t>::max();
    std::vector<std::size_t> paint_quads(
        const std::vector<ThickTriangle>& patch, const std::unordered_set<SymmetricEdge>& banned) const;

    auto begin() const { return impl.begin(); }
    auto end() const { return impl.begin(); }

private:
    void remove(SymmetricEdge edge, std::size_t from_idx)
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

private:
	std::unordered_map<SymmetricEdge, std::pair<std::size_t, std::size_t>> impl;
};
