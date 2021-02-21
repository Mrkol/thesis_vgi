#pragma once

#include <vector>
#include <stdexcept>


class DisjointSetUnion
{
    std::size_t parent_of(std::size_t idx) const
    {
        return elements[idx].replacement_idx;
    }

public:
    DisjointSetUnion(std::size_t size)
    {
        elements.reserve(size);
        while (elements.size() < size)
        {
            elements.push_back({elements.size(), 1});
        }
    }

    std::size_t get(std::size_t idx) const
    {
        while (elements[idx].replacement_idx != idx)
        {
            std::size_t tmp = parent_of(idx);
            // This is an optimization whose sideeffect is not visible from the outside.
            const_cast<DisjointSetUnion*>(this)->elements[idx].replacement_idx = parent_of(parent_of(idx));
            idx = tmp;
        }

        return idx;
    }

    // returns new representative
    std::size_t merge(std::size_t i, std::size_t j)
    {
        i = get(i);
        j = get(j);

        if (i == j)
        {
            throw std::logic_error("Patches were already merged!");
        }

        if (elements[i].size < elements[j].size)
        {
            std::swap(i, j);
        }

        elements[j].replacement_idx = i;
        elements[i].size += elements[j].size;

        return i;
    }

private:
    struct Element
    {
        std::size_t replacement_idx;
        std::size_t size;
    };

    std::vector<Element> elements;
};
