#pragma once

#include "ToPlainConverters.hpp"
#include "DataTypes.hpp"

#include <unordered_map>




template<class T>
class SurfaceHashTable
{
	auto triangle_to_verts(const ThickTriangle& t)
	{
		HashableCoords a {t.a.x, t.a.y, t.a.z};
		HashableCoords b {t.b.x, t.b.y, t.b.z};
		HashableCoords c {t.c.x, t.c.y, t.c.z};
		return std::tuple{a, b, c};
	}

public:
	static constexpr std::size_t INVALID = -1;

	std::pair<std::size_t, std::size_t> find(const SymmetricEdge& edge)
	{
		auto[beg, end] = impl.equal_range(edge);
		auto count = std::distance(beg, end);
		if (count < 1)
		{
			throw std::domain_error("Edge not in hash map!");
		}

		if (count > 2)
		{
			throw std::domain_error("Polygonal soup is not a manifold!");
		}

		if (count == 1)
		{
			// We are at the boundary
			return {beg->second, INVALID};
		}

		return {beg->second, std::next(beg)->second};
	}

	std::size_t find_not(const SymmetricEdge& edge, std::size_t idx)
	{
		auto[first, second] = find(edge);
		return first != idx ? first : second;
	}

	std::tuple<std::size_t, std::size_t, std::size_t>
		find_adjacent(const ThickTriangle& t)
	{
		auto[fst1, snd1] = find(t.a, t.b);
		auto[fst2, snd2] = find(t.b, t.c);
		auto[fst3, snd3] = find(t.c, t.a);

		if (fst1 == INVALID) { std::swap(fst1, snd1); }
		if (fst2 == INVALID) { std::swap(fst1, snd1); }
		if (fst3 == INVALID) { std::swap(fst1, snd1); }

		if (fst1 == INVALID || fst2 == INVALID || fst3 == INVALID)
		{
			throw std::domain_error("Triangle not in hash map!");
		}

		if (fst1 == fst2 && fst1 == fst3) { return {snd1, snd2, snd3}; }
		if (fst1 == fst2 && fst1 == snd3) { return {snd1, snd2, fst3}; }
		if (fst1 == snd2 && fst1 == fst3) { return {snd1, fst2, snd3}; }
		if (fst1 == snd2 && fst1 == snd3) { return {snd1, fst2, fst3}; }
		if (snd1 == fst2 && snd1 == fst3) { return {fst1, snd2, snd3}; }
		if (snd1 == fst2 && snd1 == snd3) { return {fst1, snd2, fst3}; }
		if (snd1 == snd2 && snd1 == fst3) { return {fst1, fst2, snd3}; }
		if (snd1 == snd2 && snd1 == snd3) { return {fst1, fst2, fst3}; }

		throw std::domain_error("Mesh is broken somehow!");
	}

	std::size_t add(const ThickTriangle& t)
	{
		auto[a, b, c] = triangle_to_verts(t);
		auto idx = impl.size() / 3;

		impl.emplace(SymmetricEdge{a, b}, idx);
		impl.emplace(SymmetricEdge{b, c}, idx);
		impl.emplace(SymmetricEdge{c, a}, idx);

		return idx;
	}

private:
	std::unordered_multimap<SymmetricEdge, int> impl;
};
