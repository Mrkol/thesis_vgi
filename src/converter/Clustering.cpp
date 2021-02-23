#include "SurfaceHashTable.hpp"

#include <map>
#include <set>
#include <numbers>

#include "DisjointSetUnion.hpp"
#include "DebugException.hpp"
#include "Clustering.hpp"


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
    }

    for (auto&[point, set] : data.border_graph_vertices)
    {
        std::unordered_set<std::size_t> updated_set;
        updated_set.reserve(set.size());
        for (auto idx : set)
        {
            updated_set.insert(mapping[idx]);
        }
        set = std::move(updated_set);
    }

    for (auto& i : data.accumulated_mapping)
    {
        i = mapping[i];
    }
}

Eigen::Vector4f least_squares_plane(const Eigen::Matrix4f &planarity_quadric)
{
    Eigen::Matrix3f A = planarity_quadric.block<3, 3>(0, 0);
    Eigen::Vector3f b = planarity_quadric.block<3, 1>(0, 3) * 2;
    float c = planarity_quadric(3, 3);
    Eigen::Matrix3f Z = A - b*b.transpose()/c;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver;
    solver.compute(Z);

    Eigen::Vector3f least_squares_plane_normal = solver.eigenvectors().col(0);

    float least_squares_plane_offset = -least_squares_plane_normal.dot(b)/c;

    Eigen::Vector4f result;
    result.block<3, 1>(0, 0) = least_squares_plane_normal;
    result(3, 0) = least_squares_plane_offset;
    return result;
}

float planarity_error(const Eigen::Matrix4f &planarity_quadric, std::size_t vertex_count, const Eigen::Vector4f &plane)
{
    return (plane.transpose()*planarity_quadric*plane / vertex_count).value();
}

float orientation_error(const Eigen::Matrix4f &orientation_quadric, float area, const Eigen::Vector4f &plane)
{
    return (plane.transpose()*orientation_quadric*plane / area).value();
}

float irregularity(float perimeter, float area)
{
    return perimeter*perimeter / (4*std::numbers::pi_v<float>*area);
}

float shape_error(float gamma, float gamma1, float gamma2)
{
    return (gamma - std::max(gamma1, gamma2))/gamma;
}

float after_merge_error(const Patch &first, const Patch &second, float common_perimeter)
{
    Eigen::Matrix4f plan_quad = first.planarity_quadric
                                + second.planarity_quadric;
    Eigen::Matrix4f orient_quad = first.orientation_quadric
                                  + second.orientation_quadric;

    auto plane = least_squares_plane(plan_quad);

    float perimeter = first.perimeter + second.perimeter - 2*common_perimeter;
    float area = first.area + second.area;
    std::size_t vertex_count = first.vertex_count + second.vertex_count;

    auto gamma1 = irregularity(first.perimeter, std::sqrt(first.area));
    auto gamma2 = irregularity(second.perimeter, std::sqrt(second.area));

    float gamma = irregularity(perimeter, area);

    return planarity_weight * planarity_error(plan_quad, vertex_count, plane)
           + orientation_weight * orientation_error(orient_quad, area, plane)
           + compactness_weight * shape_error(gamma, gamma1, gamma2);
}

bool merge_preserve_topological_invariants(std::size_t first, std::size_t second,
    const std::vector<Patch::BoundaryEdge>& merged_boundary, const BorderGraphVertices& border_graph_vertices)
{
    // Topological constraint: all patch intersections must be be either empty, a point, or
    // homeomorphic to a segment. If this merge results in something else, dont do it.

    std::unordered_set<std::size_t> single_point_intersections;
    for (auto it = merged_boundary.begin(); it != merged_boundary.end(); ++it)
    {
        auto prev = it == merged_boundary.begin() ? std::prev(merged_boundary.end()) : std::prev(it);

        auto patches = border_graph_vertices.find(it->starting_vertice);
        if (patches != border_graph_vertices.end())
        {
            // Approximately constant time due to typical models not
            // having verts with high numbers of adjacent edges
            for (auto patch : patches->second)
            {
                if (patch != first && patch != second && patch != Patch::NONE
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

        if (current == Patch::NONE)
        {
            ++i;
            continue;
        }

        if (already_met.contains(current) || single_point_intersections.contains(current))
        {
            return false;
        }

        while (i < merged_boundary.size() && merged_boundary[i].patch_idx == current) { ++i; }
        already_met.insert(current);
    }

    return true;
}

void rotate_boundary(std::vector<Patch::BoundaryEdge>& merged_boundary)
{
    // Fixup the boundary so that process_boundary and topinv check code is simpler
    // TODO: this is not optimal, replace vector with a custom cyclic vector
    if (merged_boundary.front().patch_idx == merged_boundary.back().patch_idx)
    {
        auto patch = merged_boundary.front().patch_idx;
        auto it = std::find_if(merged_boundary.begin(), merged_boundary.end(),
                               [patch](const Patch::BoundaryEdge& e){ return e.patch_idx != patch; });
        std::rotate(merged_boundary.begin(), it, merged_boundary.end());
    }
}

ClusteringData cluster(ClusteringData data, const std::function<bool(float, std::size_t, std::size_t)>& stopping_criterion)
{
    auto& patches = data.patches;
    auto& border_graph_vertices = data.border_graph_vertices;

    DisjointSetUnion dsu{patches.size()};

    // TODO: allocator or remove rb trees ;(
    std::multimap<float, SymmetricPair<std::size_t>> queue;
    std::unordered_map<SymmetricPair<std::size_t>, decltype(queue)::const_iterator> position_in_queue;
    position_in_queue.reserve(3*patches.size());

    auto enqueue_neighbors =
        [&data, &queue, &position_in_queue]
        (std::size_t idx, const std::function<bool(std::size_t)>& filter)
        {
            std::unordered_map<std::size_t, float> neighbors;
            for (auto[neighbor, length, _] : data.patches[idx].boundary)
            {
                if (neighbor != Patch::NONE && filter(neighbor))
                {
                    neighbors[neighbor] += length;
                }
            }

            for (auto[neighbor, common_perimeter] : neighbors)
            {
                auto total_error = after_merge_error(data.patches[idx], data.patches[neighbor], common_perimeter);
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
    while (!queue.empty() && stopping_criterion(queue.begin()->first, current_patch_count, total_patches_size))
    {
#ifdef CLUSTERING_CONSISTENCY_CHECKS
        check_consistency(data);
#endif
        auto[first, second] = queue.begin()->second;

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
        std::size_t double_common_vertex_count = 0;
        float double_common_perimeter = 0;

        {
            //TODO: dont capture so much stuff
            auto process_boundary =
                [&data, &merged_boundary, &double_common_perimeter,
                    &double_common_vertex_count]
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

                    for (auto it = begin; it != end; ++it)
                    {
                        double_common_perimeter += it->length;
                    }

                    merged_boundary.reserve(boundary.size() - std::distance(begin, end));
                    std::copy(end, boundary.end(), std::back_inserter(merged_boundary));
                    std::copy(boundary.begin(), begin, std::back_inserter(merged_boundary));
                    double_common_vertex_count += std::distance(begin, end) + 1;
                };

            process_boundary(first, second);
            process_boundary(second, first);

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
            patches[other].boundary.clear();

            patches[merged].perimeter += patches[other].perimeter;
            patches[merged].perimeter -= double_common_perimeter;

            patches[merged].has_adjacent_nones |= patches[other].has_adjacent_nones;

            patches[merged].area += patches[other].area;

            patches[merged].vertex_count += patches[other].vertex_count;
            patches[merged].vertex_count -= double_common_vertex_count;

            patches[merged].planarity_quadric += patches[other].planarity_quadric;
            patches[merged].orientation_quadric += patches[other].orientation_quadric;
        }

        // Recalculate neighbor merge errors and requeue em
        enqueue_neighbors(merged, [&neighbor_contractions_removed](std::size_t neighbor)
            {
                return neighbor_contractions_removed.contains(neighbor);
            });


        // Update border graph
        for (auto edge : patches[merged].boundary)
        {
            auto it = border_graph_vertices.find(edge.starting_vertice);
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
    SurfaceHashTable<float> edge_checker;
    for (std::size_t idx = 0; idx < data.patches.size(); ++idx)
    {
        const auto& patch = data.patches[idx];
        for (auto it = patch.boundary.begin(); it != patch.boundary.end(); ++it)
        {
            vert_checker[it->starting_vertice].insert(idx);
            auto next = std::next(it) == patch.boundary.end() ? patch.boundary.begin() : std::next(it);
            edge_checker.add({it->starting_vertice, next->starting_vertice}, idx);
        }
    }

    for (auto&[point, set] : vert_checker)
    {
        auto it = data.border_graph_vertices.find(point);
        if (set.size() >= 3
            && (it == data.border_graph_vertices.end()
                || it->second != set))
        {
            throw std::logic_error("Clustering data is inconsistent!");
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
                if (e == SymmetricEdge{boundary[found].starting_vertice, boundary[next].starting_vertice})
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
