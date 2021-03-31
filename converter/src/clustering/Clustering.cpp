#include "../SurfaceHashTable.hpp"

#include <map>
#include <set>
#include <numbers>

#include "../DisjointSetUnion.hpp"
#include "../DebugException.hpp"
#include "Clustering.hpp"


void rotate_boundary(std::vector<Patch::BoundaryEdge>& boundary)
{
    // Fixup the boundary so that process_boundary and topinv check code is simpler
    // TODO: this is not optimal, replace vector with a custom cyclic vector
    if (boundary.front().patch_idx == boundary.back().patch_idx)
    {
        auto patch = boundary.front().patch_idx;
        auto it = std::find_if(boundary.begin(), boundary.end(),
                               [patch](const Patch::BoundaryEdge& e){ return e.patch_idx != patch; });
        std::rotate(boundary.begin(), it, boundary.end());
    }
}

void reindex_clustering_data(ClusteringData& data, const std::vector<std::size_t> &mapping)
{
    for (auto& patch : data.patches)
    {
        for (auto& edge : patch.boundary)
        {
            if (edge.patch_idx != Patch::NONE)
            {
                edge.patch_idx = mapping[edge.patch_idx];
            }
        }
        rotate_boundary(patch.boundary);
    }

    for (auto&[point, set] : data.border_graph_vertices)
    {
        std::unordered_set<std::size_t> updated_set;
        updated_set.reserve(set.size());
        for (auto idx : set)
        {
            if (idx != Patch::NONE)
            {
                updated_set.insert(mapping[idx]);
            }
            else
            {
                updated_set.insert(Patch::NONE);
            }
        }
        set = std::move(updated_set);
    }

    for (auto& i : data.accumulated_mapping)
    {
        i = mapping[i];
    }
}

Vector4 least_squares_plane(const Matrix4& planarity_quadric)
{
    Matrix3 A = planarity_quadric.block<3, 3>(0, 0);
    Vector3 b = planarity_quadric.block<3, 1>(0, 3);
    FloatingNumber c = planarity_quadric(3, 3);
    Matrix3 Z = A - b*b.transpose()/c;

    Eigen::SelfAdjointEigenSolver<Matrix3> solver;
    solver.compute(Z);

    Vector3 least_squares_plane_normal = solver.eigenvectors().col(0);

    FloatingNumber least_squares_plane_offset = -least_squares_plane_normal.dot(b)/c;

    Vector4 result;
    result.block<3, 1>(0, 0) = least_squares_plane_normal;
    result(3, 0) = least_squares_plane_offset;
    return result;
}

FloatingNumber irregularity(FloatingNumber perimeter, FloatingNumber area)
{
    return perimeter*perimeter / (4*std::numbers::pi_v<FloatingNumber>*area);
}

FloatingNumber after_merge_error(const Patch& first, const Patch& second, const IntersectionData& data,
    ClusteringMetricConfig config)
{
    Matrix4 plan_quad = first.planarity_quadric
                                + second.planarity_quadric;
    Matrix4 orient_quad = first.orientation_quadric
                                  + second.orientation_quadric;

    auto plane = least_squares_plane(plan_quad);

    FloatingNumber perimeter = first.perimeter + second.perimeter - 2*data.common_perimeter;
    FloatingNumber area = first.area + second.area;
    std::size_t vertex_count = first.vertex_count + second.vertex_count - data.common_vertex_count;

    auto gamma1 = irregularity(first.perimeter, first.area);
    auto gamma2 = irregularity(second.perimeter, second.area);

    FloatingNumber gamma = irregularity(perimeter, area);


    Vector4 plane_normal = plane;
    plane_normal(3, 0) = 1;

    FloatingNumber pe = (plane.transpose()*plan_quad*plane).value() / vertex_count;
    FloatingNumber oe = (plane_normal.transpose()*orient_quad*plane_normal).value() / area;
    FloatingNumber se = (gamma - std::max(gamma1, gamma2))/gamma;
    FloatingNumber ce = perimeter*perimeter;

    return (config.planarity_weight * pe
        + config.orientation_weight * oe
        + config.irregularity_change_weight * se
        + config.compactness_weight * ce)
            / (config.planarity_weight + config.orientation_weight
                + config.irregularity_change_weight + config.compactness_weight);
}

bool merge_preserve_topological_invariants(std::size_t first, std::size_t second,
    const std::vector<Patch::BoundaryEdge>& merged_boundary, const BorderGraphVertices& border_graph_vertices)
{
    // Topological constraint: all patch intersections must be be either empty, a point, or
    // homeomorphic to a segment. Also all patches shall be homeomorphic to a disk.
    // If this merge results in something else, dont do it.

    {
        std::unordered_set<HashableCoords> unique_coords;
        for (auto& edge : merged_boundary)
        {
            unique_coords.insert(edge.starting_vertex);
        }
        if (unique_coords.size() != merged_boundary.size())
        {
            return false;
        }
    }

    std::unordered_set<std::size_t> single_point_intersections;
    for (auto it = merged_boundary.begin(); it != merged_boundary.end(); ++it)
    {
        auto prev = it == merged_boundary.begin() ? std::prev(merged_boundary.end()) : std::prev(it);

        auto patches = border_graph_vertices.find(it->starting_vertex);
        if (patches != border_graph_vertices.end())
        {
            // Approximately constant time due to typical models not
            // having verts with high numbers of adjacent edges
            for (auto patch : patches->second)
            {
                if (patch != first && patch != second
                    && patch != it->patch_idx && patch != prev->patch_idx)
                {
                    if (single_point_intersections.contains(patch))
                    {
                        // Some other patch touches both first and second by a single point, therefore the merge is invalid.
                        return false;
                    }
                    single_point_intersections.insert(patch);
                }
            }
        }
    }

    std::unordered_set<std::size_t> already_met;
    for (std::size_t i = 0; i < merged_boundary.size(); )
    {
        auto current = merged_boundary[i].patch_idx;

        if (already_met.contains(current) || single_point_intersections.contains(current))
        {
            return false;
        }

        while (i < merged_boundary.size() && merged_boundary[i].patch_idx == current) { ++i; }
        already_met.insert(current);
    }

    return true;
}

ClusteringData cluster(ClusteringData data, ClusteringConfig config)
{
    auto& patches = data.patches;
    auto& border_graph_vertices = data.border_graph_vertices;

    DisjointSetUnion dsu{patches.size()};

    // TODO: allocator or remove rb trees ;(
    std::multimap<FloatingNumber, SymmetricPair<std::size_t>> queue;
    std::unordered_map<SymmetricPair<std::size_t>, decltype(queue)::const_iterator> position_in_queue;
    position_in_queue.reserve(3*patches.size());

    auto enqueue_neighbors =
        [&data, &queue, &position_in_queue, metric_config = config.metric_config]
        (std::size_t idx, const std::function<bool(std::size_t)>& filter)
        {
            std::unordered_map<std::size_t, IntersectionData> neighbors;
            for (auto& edge : data.patches[idx].boundary)
            {
                if (edge.patch_idx != Patch::NONE && filter(edge.patch_idx))
                {
                    neighbors[edge.patch_idx].common_perimeter += edge.length;
                    neighbors[edge.patch_idx].common_vertex_count += 1;
                }
            }

            for (auto[neighbor, inter_data] : neighbors)
            {
                auto total_error = after_merge_error(data.patches[idx],
                    data.patches[neighbor], inter_data, metric_config);
                SymmetricPair<std::size_t> our_pair{idx, neighbor};

                position_in_queue.emplace(our_pair, queue.emplace(total_error, our_pair));
            }
        };


    std::size_t total_patches_size = 0;
    for (std::size_t i = 0; i < patches.size(); ++i)
    {
        total_patches_size += patches[i].total_size();

        enqueue_neighbors(i, [i](std::size_t neighbor) { return neighbor > i; });
    }

    auto remove_old_contraction = [&position_in_queue, &queue](SymmetricPair<std::size_t> contr)
    {
        auto it = position_in_queue.find(contr);
        if (it == position_in_queue.end())
        {
            // This means that the contraction got rejected due to topological invariant violation.
            return false;
        }
        queue.erase(it->second);
        position_in_queue.erase(it);
        return true;
    };

    std::size_t current_patch_count = patches.size();
    while (!queue.empty() && config.stopping_criterion(queue.begin()->first, current_patch_count, total_patches_size))
    {
#ifdef CLUSTERING_CONSISTENCY_CHECKS
        if ((current_patch_count & 0b11111) == 0)
        {
            check_consistency(data);
            check_topological_invariants(data);
        }
#endif
        SymmetricPair<std::size_t> contraction = queue.begin()->second;
        auto[first, second] = contraction;

        if (first != dsu.get(first) || second != dsu.get(second))
        {
            throw std::logic_error("Invariant violated!");
        }

        remove_old_contraction({first, second});

        auto update_boundary =
            [&dsu](std::vector<Patch::BoundaryEdge>& boundary)
            {
                for (auto& edge : boundary)
                {
                    if (edge.patch_idx != Patch::NONE)
                    {
                        edge.patch_idx = dsu.get(edge.patch_idx);
                    }
                }
                // After updating edges the front and the back might have become the same chunk,
                // therefore the fixup
                rotate_boundary(boundary);
            };

        update_boundary(patches[first].boundary);
        update_boundary(patches[second].boundary);

        // Build common boundary and check for topological invariant
        std::vector<Patch::BoundaryEdge> merged_boundary;
        IntersectionData merge_intersection_data;

        {
            auto process_boundary =
                [&data, &merged_boundary]
                    (std::size_t patch, std::size_t other)
                {
                    auto matching = [other](Patch::BoundaryEdge e) { return e.patch_idx == other; };
                    const auto &boundary = data.patches[patch].boundary;

                    auto begin = std::find_if(boundary.begin(), boundary.end(), matching);
                    auto end = std::find_if(boundary.rbegin(), boundary.rend(), matching).base();

                    if (end - begin<= 0)
                    {
                        throw std::logic_error("Trying to merge non-neighboring patches!");
                    }

                    FloatingNumber common_perimeter = 0;
                    for (auto it = begin; it != end; ++it)
                    {
                        common_perimeter += it->length;
                    }

                    merged_boundary.reserve(merged_boundary.size() + boundary.size() - std::distance(begin, end));
                    std::copy(end, boundary.end(), std::back_inserter(merged_boundary));
                    std::copy(boundary.begin(), begin, std::back_inserter(merged_boundary));
                    return IntersectionData{common_perimeter, std::size_t(std::distance(begin, end)) + 1};
                };

            merge_intersection_data = process_boundary(first, second);
            auto alt_inter_data = process_boundary(second, first);

#ifdef CLUSTERING_CONSISTENCY_CHECKS
            if ((merge_intersection_data.common_perimeter - alt_inter_data.common_perimeter)
                    / merge_intersection_data.common_perimeter > 1e-4
                || merge_intersection_data.common_vertex_count != alt_inter_data.common_vertex_count)
            {
                throw std::logic_error("Boundaries produced inconsistent intersection data!");
            }
#else
            // Suppress unused warning
            (void) alt_inter_data;
#endif

            rotate_boundary(merged_boundary);
        }

        // This assumes a fixed up boundary
        if (!merge_preserve_topological_invariants(first, second, merged_boundary, border_graph_vertices))
        {
            continue;
        }


        // We start the merge process by throwing out contractions with first/second from the queue

        std::unordered_set<std::size_t> neighbor_contractions_removed;
        {
            std::array pair{first, second};

            for (size_t i = 0; i < pair.size(); ++i)
            {
                std::unordered_set<std::size_t> neighbors;
                for (auto edge : patches[pair[i]].boundary)
                {
                    if (edge.patch_idx != Patch::NONE && edge.patch_idx != pair[1 - i])
                    {
                        neighbors.insert(edge.patch_idx);
                    }
                }

                for (auto neighbor : neighbors)
                {
                    if (remove_old_contraction({pair[i], neighbor}))
                    {
                        neighbor_contractions_removed.insert(neighbor);
                    }
                }
            }
        }

        // Do the actual merge
        --current_patch_count;
        total_patches_size -= patches[first].total_size() + patches[second].total_size();

        auto merged = dsu.merge(first, second);
        auto other = first != merged ? first : second;

        {
            patches[merged].boundary = std::move(merged_boundary);

            patches[merged].perimeter += patches[other].perimeter;
            patches[merged].perimeter -= 2 * merge_intersection_data.common_perimeter;

            patches[merged].has_vertices_adjacent_to_none |= patches[other].has_vertices_adjacent_to_none;

            patches[merged].area += patches[other].area;

            patches[merged].vertex_count += patches[other].vertex_count;
            patches[merged].vertex_count -= merge_intersection_data.common_vertex_count;

            patches[merged].planarity_quadric += patches[other].planarity_quadric;
            patches[merged].orientation_quadric += patches[other].orientation_quadric;
        }

        // Recalculate neighbor merge errors and requeue em
        enqueue_neighbors(merged, [&neighbor_contractions_removed](std::size_t neighbor)
            {
                return neighbor_contractions_removed.contains(neighbor);
            });


        // Update border graph
        for (const auto& edge : patches[merged].boundary)
        {
            auto it = border_graph_vertices.find(edge.starting_vertex);
            if (it == border_graph_vertices.end())
            {
                continue;
            }

            it->second.erase(first);
            it->second.erase(second);
            it->second.insert(merged);

            if (it->second.size() < 3)
            {
                border_graph_vertices.erase(it);
            }
        }

#ifdef CLUSTERING_CONSISTENCY_CHECKS
        for (const auto& edge : patches[merged].boundary)
        {
            // Neighbor patch update is required for other checks to work
            if (edge.patch_idx != Patch::NONE)
            {
                update_boundary(patches[edge.patch_idx].boundary);
            }
        }
#endif

        total_patches_size += patches[merged].total_size();
        patches[other] = {};
    }

    // Transform answer back into desired format

    std::vector<std::size_t> resulting_patch_ids;
    {
        std::set<std::size_t> resulting_patch_ids_set;
        for (std::size_t i = 0; i < patches.size(); ++i)
        {
            resulting_patch_ids_set.insert(dsu.get(i));
        }
        resulting_patch_ids.reserve(resulting_patch_ids_set.size());
        for (auto idx : resulting_patch_ids_set)
        {
            resulting_patch_ids.push_back(idx);
        }
    }

    std::vector<std::size_t> old_to_new;
    old_to_new.resize(patches.size(), 0);
    for (std::size_t i = 0; i < patches.size(); ++i)
    {
        old_to_new[i] = std::lower_bound(resulting_patch_ids.begin(), resulting_patch_ids.end(), dsu.get(i))
                        - resulting_patch_ids.begin();
    }

    ClusteringData result;
    result.patches.reserve(resulting_patch_ids.size());
    for (auto id : resulting_patch_ids)
    {
        result.patches.push_back(std::move(patches[id]));
    }
    result.border_graph_vertices = std::move(border_graph_vertices);
    result.accumulated_mapping = std::move(data.accumulated_mapping);

    // We've thrown out some patches, the indexing changed, need to update it.
    reindex_clustering_data(result, old_to_new);

    return result;
}

void check_consistency(const ClusteringData& data)
{
    BorderGraphVertices vert_checker;
    SurfaceHashTable<FloatingNumber> edge_checker;
    for (std::size_t idx = 0; idx < data.patches.size(); ++idx)
    {
        const auto& patch = data.patches[idx];
        FloatingNumber perimeter = 0;
        for (auto it = patch.boundary.begin(); it != patch.boundary.end(); ++it)
        {
            vert_checker[it->starting_vertex].insert(idx);
            auto next = std::next(it) == patch.boundary.end() ? patch.boundary.begin() : std::next(it);

            if (it->patch_idx == Patch::NONE)
            {
                vert_checker[it->starting_vertex].insert(Patch::NONE);
                vert_checker[next->starting_vertex].insert(Patch::NONE);
            }

            SymmetricEdge edge{it->starting_vertex, next->starting_vertex};
            edge_checker.add(edge, idx);
            perimeter += length(edge);
        }

        if (std::abs(perimeter - patch.perimeter) / perimeter > 1e-3)
        {
            throw std::logic_error(
                "Clustering data is inconsistent! Perimeter doesn't match the actual boundary perimeter!");
        }
    }

    for (auto&[point, set] : vert_checker)
    {
        auto it = data.border_graph_vertices.find(point);
        if (set.size() >= 3
            && (it == data.border_graph_vertices.end()
                || it->second != set))
        {
            throw std::logic_error("Clustering data is inconsistent! Border graph vertices are missing!");
        }
    }

    for (auto&[point, set] : data.border_graph_vertices)
    {
        auto it = vert_checker.find(point);
        if (it == vert_checker.end() || it->second != set)
        {
            throw std::logic_error("Clustering data is inconsistent! Too many border graph vertices!");
        }
    }

    for (auto&[edge, pair] : edge_checker)
    {
        auto check = [&, e = edge](std::size_t idx, std::size_t neighbor)
        {
            auto& boundary = data.patches[idx].boundary;
            std::size_t found = 0;
            while (found != boundary.size())
            {
                auto next = found + 1 == boundary.size() ? 0 : found + 1;
                if (e == SymmetricEdge{boundary[found].starting_vertex, boundary[next].starting_vertex})
                {
                    break;
                }
            }

            if (found == boundary.size()
                || boundary[found].patch_idx != neighbor)
            {
                throw std::logic_error("Clustering data is inconsistent!");
            }
        };

        check(pair.first, pair.second);
        check(pair.second, pair.first);
    }
}

void check_topological_invariants(const ClusteringData& data)
{
    for (std::size_t idx = 0; idx < data.patches.size(); ++idx)
    {
        const auto& boundary = data.patches[idx].boundary;

        // Check whether we look like a disc
        {
            std::unordered_set<HashableCoords> unique_coords;
            for (auto& edge : boundary)
            {
                unique_coords.insert(edge.starting_vertex);
            }
            if (unique_coords.size() != boundary.size())
            {
                throw std::logic_error("Patch " + std::to_string(idx) + " has self-intersecting boundary");
            }
        }


        // check that intersections are segments

        std::unordered_set<std::size_t> single_point_intersections;
        for (auto it = boundary.begin(); it != boundary.end(); ++it)
        {
            auto prev = it == boundary.begin() ? std::prev(boundary.end()) : std::prev(it);

            auto patches = data.border_graph_vertices.find(it->starting_vertex);
            if (patches != data.border_graph_vertices.end())
            {
                // Approximately constant time due to typical models not
                // having verts with high numbers of adjacent edges
                for (auto patch : patches->second)
                {
                    if (patch != Patch::NONE && patch != idx && patch != it->patch_idx && patch != prev->patch_idx)
                    {
                        if (single_point_intersections.contains(patch))
                        {
                            // Some other patch touches both first and second by a single point, therefore the merge is invalid.
                           throw std::logic_error("Patches " + std::to_string(idx) + " and " + std::to_string(patch) +
                               " intersect by two \"isolated\" points!");
                        }
                        single_point_intersections.insert(patch);
                    }
                }
            }
        }

        std::unordered_set<std::size_t> already_met;
        for (std::size_t i = 0; i < boundary.size(); )
        {
            auto current = boundary[i].patch_idx;

            if (current == Patch::NONE)
            {
                ++i;
                continue;
            }

            if (already_met.contains(current))
            {
                throw std::logic_error("Patches " + std::to_string(idx) + " and " + std::to_string(current) +
                    " intersect by 2 segments!");
            }

            if (single_point_intersections.contains(current))
            {
                throw std::logic_error("Patches " + std::to_string(idx) + " and " + std::to_string(current) +
                    " intersect by a segment and a point!");
            }

            while (i < boundary.size() && boundary[i].patch_idx == current) { ++i; }
            already_met.insert(current);
        }
    }
}
