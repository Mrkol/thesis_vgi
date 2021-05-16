#include "HierarchicalAtlas.hpp"

#include <unordered_set>
#include <fstream>
#include <utility>
#include <deque>

#include "Utility.hpp"


GeometryImage::GeometryImage(const std::filesystem::path& path)
{
    std::ifstream in{path, std::ios_base::binary};

    data.resize(file_size(path) / sizeof(Eigen::Vector3f));
    in.read(reinterpret_cast<char*>(data.data()),
        static_cast<std::streamsize>(data.size() * sizeof(Eigen::Vector3f)));

    width = static_cast<std::size_t>(std::sqrt(data.size()));
}

Eigen::Vector3f& GeometryImage::operator ()(std::size_t x, std::size_t y)
{
    AD_HOC_ASSERT(x < width && y < width, "out of bounds");
    return data[y*width + x];
}

const Eigen::Vector3f& GeometryImage::operator ()(std::size_t x, std::size_t y) const
{
    return const_cast<GeometryImage&>(*this).operator ()(x, y);
}

GeometryImage GeometryImage::upscale(std::size_t newsize) const
{
    GeometryImage result;

    result.data.resize(newsize * newsize);
    result.width = newsize;

    static constexpr auto lerp = []<class T>(T c1, T c2, float k)
        {
            return (1 - k)*c1 + k*c2;
        };
    static constexpr auto blerp = []<class T>(T c11, T c21, T c12, T c22, float kx, float ky)
        {
            return lerp(lerp(c11, c21, kx), lerp(c12, c22, kx), ky);
        };


    for (std::size_t x = 0; x < newsize; ++x)
    {
        for (std::size_t y = 0; y < newsize; ++y)
        {
            float x_ = float(x * width) / float(newsize);
            float y_ = float(y * width) / float(newsize);

            auto xi = std::size_t(x_);
            auto yi = std::size_t(y_);

            result(x, y) =
                blerp
                ( operator ()(xi,     yi)
                , operator ()(xi + 1, yi)
                , operator ()(xi,     yi + 1)
                , operator ()(xi + 1, yi + 1)
                , x_ - float(xi)
                , y_ - float(yi)
                );
        }
    }

    return result;
}

QuadtreeNode::QuadtreeNode(QuadtreeNode::CreateInfo info)
    : children{nullptr}
    , size{info.size}
    , offset{info.offset}
    , bounding_box_start{Eigen::Vector3f::Constant(std::numeric_limits<float>::max())}
    , bounding_box_end{Eigen::Vector3f::Constant(-std::numeric_limits<float>::max())}
    , min_tessellation{info.levels_left}
    , max_tessellation{info.levels_left + info.GIs.size() - 1}
    , mip_errors(info.GIs.size(), 0)
{
    if (info.levels_left > 0)
    {
        float size_ = size/2;
        std::array offsets{
            Eigen::Vector2f{0, 0},
            Eigen::Vector2f{size_, 0},
            Eigen::Vector2f{0, size_},
            Eigen::Vector2f{size_, size_},
        };

        for (std::size_t i = 0; i < 4; ++i)
        {
            children[i] = std::make_unique<QuadtreeNode>(CreateInfo{
                info.levels_left - 1,
                size_,
                offset + offsets[i],
                info.GIs,
                info.upsampled_GIs
            });
            children[i]->parent = this;

            for (std::size_t j = 0; j < mip_errors.size(); ++j)
            {
                mip_errors[j] = std::max(mip_errors[j], children[i]->mip_errors[j]);
            }
            bounding_box_start = bounding_box_start.cwiseMin(children[i]->bounding_box_start);
            bounding_box_end = bounding_box_end.cwiseMax(children[i]->bounding_box_end);
        }

    }
    else
    {
        using IntVec = Eigen::Matrix<std::size_t, 2, 1>;

        for (std::size_t i = 1; i < info.GIs.size(); ++i)
        {
            auto& gi = info.upsampled_GIs[i - 1];
            auto& original_gi = info.GIs[0];

            auto subregion_size = std::size_t(float(original_gi.size()) * size);
            auto subregion_offset = (float(original_gi.size()) * offset).cast<std::size_t>().eval();

            float& error = mip_errors[i];
            for (std::size_t x = 0; x < subregion_size; ++x)
            {
                for (std::size_t y = 0; y < subregion_size; ++y)
                {
                    auto p = gi(subregion_offset.x() + x, subregion_offset.y() + y);
                    auto po = original_gi(subregion_offset.x() + x, subregion_offset.y() + y);
                    error = std::max(error, (p - po).norm());

                    bounding_box_start = bounding_box_start.cwiseMin(po);
                    bounding_box_end = bounding_box_end.cwiseMax(po);
                }
            }
        }
    }
}

AtlasPatch::AtlasPatch(CreateInfo info)
{
    gis.reserve(info.max_mip - info.min_mip + 1);
    for (std::size_t i = info.min_mip; i <= info.max_mip; ++i)
    {
        gis.emplace_back(
            info.images_folder / "positions" / (std::string{info.name_prefix} + ":" + std::to_string(i))
        );
    }

    std::vector<GeometryImage> upsampled;
    upsampled.reserve(gis.size() - 1);

    for (std::size_t i = 1; i < gis.size(); ++i)
    {
        upsampled.emplace_back(gis[i].upscale(gis[0].size()));
    }

    root = std::make_unique<QuadtreeNode>(QuadtreeNode::CreateInfo{
        info.min_mip,
        1.f, Eigen::Vector2f::Zero(),
        gis, upsampled
    });

    {
        auto& gi = gis[0];
        auto sz = gi.size() - 1;
        max_res_corners =
            { gi(0, 0)
            , gi(sz, 0)
            , gi(sz, sz)
            , gi(0, sz)
            };
    }
}

HierarchicalAtlas::HierarchicalAtlas(std::filesystem::path images_folder)
{
    std::unordered_set<std::string> prefixes;
    min_mip = 1000; // 2^1000 is a lot
    max_mip = 0;
    for (auto& entry : std::filesystem::directory_iterator(images_folder / "positions"))
    {
        auto name = entry.path().filename().string();
        auto pos = name.rfind(':');
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
}

HierarchyCut HierarchicalAtlas::default_cut()
{
    HierarchyCut result;

    for (std::size_t i = 0; i < patches.size(); ++i)
    {
        result.elements.emplace(patches[i].root.get(),
            HierarchyCut::CutElement{i, min_mip, {min_mip, min_mip, min_mip, min_mip}, &patches[i]});
    }

    return result;
}

void HierarchyCut::split(QuadtreeNode* node)
{
    auto it = elements.find(node);

    if (it == elements.end() || node->children[0] == nullptr)
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
        elements.emplace(node->children[i].get(), copy);
    }
}

std::vector<std::pair<QuadtreeNode*, HierarchyCut::CutElement>> HierarchyCut::dump() const
{
    std::vector<std::pair<QuadtreeNode*, HierarchyCut::CutElement>> result;
    for (const auto& pair : elements)
    {
        result.emplace_back(pair);
    }
    return result;
}

const HierarchyCut::CutElement& HierarchyCut::get_data(QuadtreeNode* node) const
{
    return elements.at(node);
}

QuadtreeNode* find_side_neighbor_search_start(QuadtreeNode* node, AtlasPatch* patch, std::size_t side)
{
    std::array bad_child_indices{
        std::make_pair(0, 1), // side 0
        std::make_pair(1, 3), // side 1
        std::make_pair(2, 3), // side 2
        std::make_pair(0, 2), // side 3
    };

    while (node->parent != nullptr
        && (node->parent->children[bad_child_indices[side].first].get() == node
            || node->parent->children[bad_child_indices[side].second].get() == node))
    {
        node = node->parent;
    }

    if (node->parent == nullptr)
    {
        auto neighbor_patch = patch->neighbors[side];
        if (neighbor_patch != nullptr)
        {
            return neighbor_patch->root.get();
        }

        return nullptr;
    }

    std::array lookup{
        std::array{-1, -1,  0,  1},
        std::array{ 1, -1,  3, -1},
        std::array{ 2,  3, -1, -1},
        std::array{-1,  0, -1,  2}
    };

    for (std::size_t i = 0; i < 4; ++i)
    {
        if (node->parent->children[i].get() == node)
        {
            return node->parent->children[lookup[side][i]].get();
        }
    }

    AD_HOC_PANIC("Something went very wrong");
}

bool segments_intersect(float a, float b, float c, float d)
{
    return std::max(a, c) <= std::min(b, d);
}

bool nodes_touch(QuadtreeNode* first, QuadtreeNode* second, Eigen::Matrix3f transport)
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
std::vector<QuadtreeNode*> HierarchyCut::find_side_neighbors(
    QuadtreeNode* node, std::size_t side)
{
    AtlasPatch* patch = elements.at(node).root;

    std::vector<QuadtreeNode*> result;

    auto start = find_side_neighbor_search_start(node, patch, side);

    if (start == nullptr)
    {
        return result;
    }


    Eigen::Matrix3f transport = Eigen::Matrix3f::Identity();
    if (patch->neighbors[side] != nullptr
        && start == patch->neighbors[side]->root.get())
    {
        std::array xs{0.f, 1.f, 0.f, -1.f};
        std::array ys{-1.f, 0.f, 1.f, 0.f};
        transport =
            ( Eigen::Translation2f(xs[side], ys[side])
            * Eigen::Translation2f(.5f, .5f)
            * Eigen::Rotation2Df(float(EIGEN_PI/2 * patch->neighbor_rotation_difference[side]))
            * Eigen::Translation2f(-.5f, -.5f)
            ).matrix().array().round();
    }

    std::deque<QuadtreeNode*> frontier;
    frontier.push_back(start);

    while (!frontier.empty())
    {
        auto current = frontier.front();
        //
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

        for (auto& child : current->children)
        {
            if (child)
            {
                frontier.push_back(child.get());
            }
        }
    }

    return result;
}

void HierarchyCut::set_mip(QuadtreeNode* node, std::size_t mip)
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
            continue;
        }

        QuadtreeNode* lengthiest = node;
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
            if (data.root->neighbors[i] == elements[lengthiest].root)
            {
                node_side += data.root->neighbor_rotation_difference[i];
            }
        }
        else
        {
            if (data.root->neighbors[i] == elements[neighbors[0]].root)
            {
                neighbor_side -= data.root->neighbor_rotation_difference[i];
            }
        }

        recalculate_side_mips(lengthiest, node_side % 4, neighbor_side % 4);
    }
}

void HierarchyCut::recalculate_side_mips(QuadtreeNode* node, std::size_t node_side, std::size_t neighbor_side)
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

