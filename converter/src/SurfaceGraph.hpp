#pragma once

#include <unordered_set>
#include <unordered_map>
#include <vector>

#include "DataTypes.hpp"


// TODO: This is not a good abstraction, redesign
class SurfaceGraph
{
public:
    explicit SurfaceGraph(const std::vector<ThickTriangle>& triangles);

    [[nodiscard]] const std::unordered_set<size_t>& adjacent_to(size_t idx) const;

    [[nodiscard]] FloatingNumber get_distance(size_t u, size_t v) const;

    [[nodiscard]] size_t index_of(HashableCoords point) const;

    [[nodiscard]] HashableCoords coords(size_t idx) const;

    [[nodiscard]] bool is_edge(size_t u, size_t v);

    [[nodiscard]] size_t vertex_count() const;

    // Look at the picture where this thing is used
    void split_edge(HashableCoords u, HashableCoords v,
        HashableCoords a, std::optional<HashableCoords> b, HashableCoords m);

private:
    std::unordered_map<HashableCoords, size_t> vertex_indices;
    std::vector<HashableCoords> vertices;

    std::vector<std::unordered_set<size_t>> adjacency_list;
};
