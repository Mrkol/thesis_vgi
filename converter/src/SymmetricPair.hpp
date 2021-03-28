#pragma once

#include <utility>
#include <functional>

template<class T>
struct SymmetricPair
{
	T first;
	T second;
};

template<class T>
bool operator==(const SymmetricPair<T>& left, const SymmetricPair<T>& right)
{
	return (left.first == right.first && left.second == right.second)
		|| (left.first == right.second && left.second == right.first);
}

namespace std
{

template<class T>
struct hash<SymmetricPair<T>>
{
	std::size_t operator()(SymmetricPair<T> pair) const
	{
		auto h = std::hash<T>();
		return h(pair.first) ^ h(pair.second);
	}
};

}
