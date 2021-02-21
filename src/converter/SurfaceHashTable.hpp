#pragma once

#include "ToPlainConverters.hpp"
#include "DataTypes.hpp"

#include <unordered_map>



template<class T>
class SurfaceHashTable
{
public:
	static constexpr std::size_t INVALID = std::numeric_limits<std::size_t>::max();

	std::pair<std::size_t, std::size_t> find(const SymmetricEdge& edge)
	{
		auto it = impl.find(edge);

		if (it == impl.end())
		{
			throw std::logic_error("Edge not in hash map!");
		}

		return it->second;
	}

	std::size_t find_not(const SymmetricEdge& edge, std::size_t idx)
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

    auto begin() const { return impl.begin(); }
    auto end() const { return impl.begin(); }

private:
	std::unordered_map<SymmetricEdge, std::pair<std::size_t, std::size_t>> impl;
};
