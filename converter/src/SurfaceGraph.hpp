#pragma once

#include <unordered_set>
#include <unordered_map>
#include <optional>
#include <vector>
#include <function2/function2.hpp>

#include "DataTypes.hpp"


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

    /**
     * Splits the edge (u, v) into two edges, (u, m) and (m, v), where m is the midpoint of it
     * @param u Edge start
     * @param v Edge end
     * @param a Vertex which completes the triangle (u, v, a)
     * @param b Vertex which completes the other triangle (u, v, b), if there is one
     * @param m Midpoint of (u, v)
     */
    void split_edge(HashableCoords u, HashableCoords v,
        HashableCoords a, std::optional<HashableCoords> b, HashableCoords m);

    /**
     * Finds shortest path to each vertex from the specified one
     * @param start Vertex to start from
     * @return Ith element is the distance to ith vertex
     */
    std::vector<FloatingNumber> dijkstra(std::size_t start) const;

    /**
     * Reconstructs the shortest path from start to end using specified distances
     * @param distances Result of a dijkstra call with the same start parameter
     * @param start Starting vertex for the path
     * @param end End vertex for the path
     * @return Array of vertex ids for the shortest path, starting from start and ending in end
     */
    [[nodiscard]] std::vector<std::size_t> recontstruct_shortest_path(
        const std::vector<FloatingNumber>& distances,
        std::size_t start, std::size_t end) const;

    /**
     * Finds shortest path from start to finish via A*
     * @param start Vertex to start from
     * @param finish Target vertex
     * @return Array of vertex ids for the shortest path, starting from start and ending in end
     */
    [[nodiscard]] std::vector<std::size_t> a_star(std::size_t start, std::size_t finish) const;

    /**
     * Finds shortest path from start to finish via A*, but allows weighing edge lengths with an external factor.
     * New edge weight is old / scaling_factor(new_vertex). Useful for keeping the path away from the edges of a patch.
     * @param start Vertex to start from
     * @param finish Target vertex
     * @return Array of vertex ids for the shortest path, starting from start and ending in end
     */
    [[nodiscard]] std::vector<std::size_t> scaled_a_star(std::size_t start, std::size_t finish,
        fu2::unique_function<FloatingNumber(std::size_t)> scale_factor) const;

private:
    std::unordered_map<HashableCoords, size_t> vertex_indices;
    std::vector<HashableCoords> vertices;

    std::vector<std::unordered_set<size_t>> adjacency_list;
};
