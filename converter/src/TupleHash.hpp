#pragma once

#include <tuple>
#include <functional>


namespace std
{

namespace
{

struct HashWrapper
{
	std::size_t hash;
};

// Clang doesnt recognize this as being used below for some reason...
[[maybe_unused]] inline HashWrapper operator^(HashWrapper a, HashWrapper b)
{
	return {a.hash ^ (b.hash * 0x9e3779b9 + (a.hash << 6) + (a.hash >> 2))};
}

}

template<class... Ts>
struct hash<std::tuple<Ts...>>
{
	std::size_t operator()(const std::tuple<Ts...>& tuple) const
	{
		return std::apply([](const Ts&... ts)
			{
				return (HashWrapper{std::hash<Ts>{}(ts)} ^ ...).hash;
			}, tuple);
	}
};

}
