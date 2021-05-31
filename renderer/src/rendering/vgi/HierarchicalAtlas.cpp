#include "HierarchicalAtlas.hpp"

#include <unordered_set>
#include <fstream>
#include <utility>
#include <deque>

#include "Utility.hpp"
#include "AtlasPatch.hpp"



NodeHandle find_side_neighbor_search_start(NodeHandle node, std::size_t side)
{
    std::array bad_child_indices{
        std::make_pair(0, 1), // side 0
        std::make_pair(1, 3), // side 1
        std::make_pair(2, 3), // side 2
        std::make_pair(0, 2), // side 3
    };

    while (node.has_parent()
        && (node.parent().child(bad_child_indices[side].first) == node
            || node.parent().child(bad_child_indices[side].second) == node))
    {
        node = node.parent();
    }

    if (!node.has_parent())
    {
        if (auto neighbor_patch = node.patch().neighbors[side])
        {
            return NodeHandle::root_of(neighbor_patch);
        }

        return {};
    }

    std::array lookup{
        std::array{-1, -1,  0,  1},
        std::array{ 1, -1,  3, -1},
        std::array{ 2,  3, -1, -1},
        std::array{-1,  0, -1,  2}
    };

    for (std::size_t i = 0; i < 4; ++i)
    {
        if (node.parent().child(i) == node)
        {
            return node.parent().child(lookup[side][i]);
        }
    }

    AD_HOC_PANIC("Something went very wrong");
}

HierarchicalAtlas::HierarchicalAtlas(const std::filesystem::path& images_folder)
{
    std::unordered_set<std::string> prefixes;
    min_mip = 1000; // 2^1000 is a lot
    max_mip = 0;
    for (auto& entry : std::filesystem::directory_iterator(images_folder))
    {
        auto name = entry.path().filename().string();
        auto pos = name.rfind(',');
        prefixes.emplace(name.substr(0, pos));
        std::size_t mip = std::stoul(name.substr(pos + 1));
        min_mip = std::min(min_mip, mip);
        max_mip = std::max(max_mip, mip);
    }

    patches.reserve(prefixes.size());
    for (auto& prefix : prefixes)
    {
        patches.emplace_back(AtlasPatch::CreateInfo{images_folder, prefix, min_mip, max_mip});
    }

    // O(n^2) is ok here as n < 100
    // TODO: or maybe not?
    for (auto& patch1 : patches)
    {
        for (auto& patch2 : patches)
        {
            if (&patch1 == &patch2)
            {
                continue;
            }

            std::array side_to_corners{
                std::make_pair(0, 1),
                std::make_pair(1, 2),
                std::make_pair(2, 3),
                std::make_pair(3, 0)
            };

            for (std::size_t i = 0; i < 4; ++i)
            {
                for (std::size_t j = 0; j < 4; ++j)
                {
                    if ((patch1.max_res_corners[side_to_corners[i].first]
                            - patch2.max_res_corners[side_to_corners[j].second]).norm() < 1e-3
                        &&
                        (patch1.max_res_corners[side_to_corners[i].second]
                            - patch2.max_res_corners[side_to_corners[j].first]).norm() < 1e-3)
                    {
                        patch1.neighbors[i] = &patch2;
                        patch1.neighbor_rotation_difference[i] = (i + 6 - j) % 4;
                        patch2.neighbors[j] = &patch1;
                        patch2.neighbor_rotation_difference[j] = (j + 6 - i) % 4;
                    }
                }
            }
        }
    }

    for (auto& patch : patches)
    {
        for (NodeIdx i = 0; i < patch.nodes.size(); ++i)
        {
            for (std::size_t j = 0; j < 4; ++j)
            {
                patch.nodes[i].neighbor_search_starts[j] = find_side_neighbor_search_start(NodeHandle{i, &patch}, j);
            }
        }
    }
}

HierarchyCut HierarchicalAtlas::default_cut()
{
    HierarchyCut result;

    for (std::size_t i = 0; i < patches.size(); ++i)
    {
        result.elements.emplace(NodeHandle::root_of(&patches[i]),
            HierarchyCut::CutElement{i, min_mip, {min_mip, min_mip, min_mip, min_mip}});
    }

    return result;
}

void HierarchyCut::split(NodeHandle node)
{
    auto it = elements.find(node);

    if (it == elements.end() || !it->first.has_children())
    {
        return;
    }

    CutElement data = it->second;
    elements.erase(it);

    --data.mip;
    for (auto& m : data.side_mip)
    {
        --m;
    }

    std::array inside_edges{
        std::make_pair(1, 2),
        std::make_pair(2, 3),
        std::make_pair(0, 1),
        std::make_pair(3, 0),
    };

    for (std::size_t i = 0; i < 4; ++i)
    {
        auto copy = data;
        copy.side_mip[inside_edges[i].first] = data.mip;
        copy.side_mip[inside_edges[i].second] = data.mip;
        elements.emplace(node.child(i), copy);
    }
}

std::vector<std::pair<NodeHandle, HierarchyCut::CutElement>> HierarchyCut::dump() const
{
    std::vector<std::pair<NodeHandle, HierarchyCut::CutElement>> result;
    for (const auto& pair : elements)
    {
        result.emplace_back(pair);
    }
    return result;
}

bool segments_intersect(float a, float b, float c, float d)
{
    return std::max(a, c) <= std::min(b, d);
}

bool nodes_touch(NodeHandle first, NodeHandle second, const Eigen::Matrix3f& transport)
{
    Eigen::Vector2f first_start_transformed =
        (transport * Eigen::Vector3f(first->offset.x(), first->offset.y(), 1))
        .block<2, 1>(0, 0);

    Eigen::Vector2f first_end_transformed =
        (transport * Eigen::Vector3f(first->offset.x() + first->size, first->offset.y() + first->size, 1))
        .block<2, 1>(0, 0);

    Eigen::Vector2f first_start = first_start_transformed.cwiseMin(first_end_transformed);
    Eigen::Vector2f first_end = first_start_transformed.cwiseMax(first_end_transformed);

    return
        (segments_intersect(
            first_start.x(), first_end.x(),
            second->offset.x(), second->offset.x() + second->size)
        && (first_start.y() == second->offset.y() + second->size
                || second->offset.y() == first_end.y()))
        || (segments_intersect(
            first_start.y(), first_end.y(),
            second->offset.y(), second->offset.y() + second->size)
        && (first_start.x() == second->offset.x() + second->size
                || second->offset.x() == first_end.x()));
}

/**
 * See AtlasPatch::neighbors for the meaning of side param.
 */
std::vector<NodeHandle> HierarchyCut::find_side_neighbors(
    const NodeHandle node, std::size_t side)
{
    const AtlasPatch& patch = node.patch();

    std::vector<NodeHandle> result;

    auto start = node->neighbor_search_starts[side];

    if (!start.valid())
    {
        return result;
    }

    Eigen::Matrix3f transport = Eigen::Matrix3f::Identity();
    if (patch.neighbors[side] != nullptr
        && start == NodeHandle::root_of(patch.neighbors[side]))
    {
        std::array xs{0.f, 1.f, 0.f, -1.f};
        std::array ys{-1.f, 0.f, 1.f, 0.f};
        transport =
            ( Eigen::Translation2f(xs[side], ys[side])
            * Eigen::Translation2f(.5f, .5f)
            * Eigen::Rotation2Df(float(EIGEN_PI/2 * patch.neighbor_rotation_difference[side]))
            * Eigen::Translation2f(-.5f, -.5f)
            ).matrix().array().round();
    }

    std::deque<NodeHandle> frontier;
    frontier.push_back(start);

    while (!frontier.empty())
    {
        auto current = frontier.front();
        frontier.pop_front();

        if (!nodes_touch(current, node, transport))
        {
            continue;
        }

        if (elements.contains(current))
        {
            result.push_back(current);
            continue;
        }

        if (current.has_children())
        {
            for (std::size_t i = 0; i < 4; ++i)
            {
                frontier.push_back(current.child(i));
            }
        }
    }

    return result;
}

void HierarchyCut::set_mip(NodeHandle node, std::size_t mip)
{
    auto& data = elements.at(node);
    if (data.mip == mip)
    {
        return;
    }

    data.mip = mip;

    for (std::size_t i = 0; i < 4; ++i)
    {
        auto neighbors = find_side_neighbors(node, i);

        if (neighbors.empty())
        {
            elements.at(node).side_mip[i] = mip;
            continue;
        }

        NodeHandle lengthiest = node;
        for (auto neighbor : neighbors)
        {
            if (neighbor->size > lengthiest->size)
            {
                lengthiest = neighbor;
            }
        }

        std::size_t node_side = i + 4;
        std::size_t neighbor_side = i + 6;

        if (lengthiest != node)
        {
            std::swap(node_side, neighbor_side);
            if (node.patch().neighbors[i] == &lengthiest.patch())
            {
                node_side += node.patch().neighbor_rotation_difference[i];
            }
        }
        else
        {
            if (node.patch().neighbors[i] == &neighbors[0].patch())
            {
                neighbor_side -= node.patch().neighbor_rotation_difference[i];
            }
        }

        recalculate_side_mips(lengthiest, node_side % 4, neighbor_side % 4);
    }
}

void HierarchyCut::recalculate_side_mips(NodeHandle node, std::size_t node_side, std::size_t neighbor_side)
{
    auto neighbors = find_side_neighbors(node, node_side);

    std::size_t min = elements.at(node).mip - node->min_tessellation;
    for (auto neighbor : neighbors)
    {
        min = std::min(min, elements[neighbor].mip - neighbor->min_tessellation);
    }

    for (auto neighbor : neighbors)
    {
        elements.at(neighbor).side_mip[neighbor_side] = neighbor->min_tessellation + min;
    }

    elements.at(node).side_mip[node_side] = node->min_tessellation + min;
}

std::vector<NodeHandle> HierarchyCut::get_nodes() const
{
    std::vector<NodeHandle> result;
    result.reserve(elements.size());
    for (auto& pair : elements)
    {
        result.push_back(pair.first);
    }
    return result;
}
