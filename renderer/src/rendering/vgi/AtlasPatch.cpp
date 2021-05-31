#include "AtlasPatch.hpp"



NodeHandle NodeHandle::child(std::size_t child) const { return NodeHandle{AtlasPatch::child_of(idx_, child), patch_}; }
bool NodeHandle::has_children() const { return patch_->has_children(idx_); }

NodeHandle NodeHandle::parent() const { return NodeHandle{AtlasPatch::parent_of(idx_), patch_}; }
bool NodeHandle::has_parent() const { return AtlasPatch::has_parent(idx_); }

AtlasNode& NodeHandle::operator*() { return patch_->nodes.at(idx_); }
const AtlasNode& NodeHandle::operator*() const { return patch_->nodes.at(idx_); }



float AtlasNode::projected_screenspace_area(
    const Eigen::Matrix4f& model, const Eigen::Matrix4f& view, const Eigen::Matrix4f& projection) const
{
    using namespace Eigen;
    Vector3f d = bounding_box_end - bounding_box_start;

    Vector4f o = view * model * bounding_box_start.homogeneous();
    Vector4f dx = view * model * Vector4f(d.x(), 0, 0, 0);
    Vector4f dy = view * model * Vector4f(0, d.y(), 0, 0);
    Vector4f dz = view * model * Vector4f(0, 0, d.z(), 0);

    Vector2f min{std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
    Vector2f max = -min;
    for (std::size_t i = 0; i < 8; ++i)
    {
        Vector4f p = projection * (o + dx*!!(i & 1) + dy*!!(i & 2) + dz*!!(i & 4));
        Vector2f s{p.x() / p.w(), p.y() / p.w()};
        min = min.cwiseMin(s);
        max = max.cwiseMax(s);
    }

    if ((max.array() < -1).any() || (min.array() > 1).any())
    {
        return 0;
    }

    max = max.cwiseMin(Vector2f{1, 1});
    min = min.cwiseMax(Vector2f{-1, -1});

    return std::abs((max - min).prod())/4;
}

AtlasPatch::AtlasPatch(const CreateInfo& info)
{
    gis.reserve(info.max_mip - info.min_mip + 1);
    for (std::size_t i = info.min_mip; i <= info.max_mip; ++i)
    {
        gis.emplace_back(
            info.images_folder / (std::string{info.name_prefix} + "," + std::to_string(i))
        );
    }

    std::vector<GeometryImage> upsampled;
    upsampled.reserve(gis.size() - 1);

    for (std::size_t i = 1; i < gis.size(); ++i)
    {
        upsampled.emplace_back(gis[i].upscale(gis[0].size()));
    }

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

    // Min_mip is a sensible depth as in the worst case scenario we'll have to divide the minimal size
    // GI into 4^depth parts
    std::size_t depth = info.min_mip;
    std::size_t target_node_count = ((1ull << (2*depth)) - 1)/3;

    nodes.reserve(target_node_count);
    nodes.push_back(AtlasNode{
        1.f, Eigen::Vector2f::Zero(), info.min_mip, info.min_mip + gis.size() - 1
    });

    for (std::size_t i = 0; nodes.size() < target_node_count; ++i)
    {
        float size = nodes[i].size / 2;
        std::array offsets{
            Eigen::Vector2f{0, 0},
            Eigen::Vector2f{size, 0},
            Eigen::Vector2f{0, size},
            Eigen::Vector2f{size, size},
        };

        for (std::size_t j = 0; j < 4; ++j)
        {
            nodes.push_back(AtlasNode{
                size, nodes[i].offset + offsets[j], nodes[i].min_tessellation - 1, nodes[i].max_tessellation - 1
            });
        }
    }

    for (std::size_t i = 0; i < nodes.size(); ++i)
    {
        std::size_t idx = nodes.size() - i - 1;
        auto& node = nodes[idx];

        node.mip_errors.resize(gis.size());

        if (!has_children(idx))
        {
            for (std::size_t j = 1; j < gis.size(); ++j)
            {
                auto& gi = upsampled[j - 1];
                auto& original_gi = gis[0];

                auto subregion_size = std::size_t(float(original_gi.size()) * node.size);
                auto subregion_offset = (float(original_gi.size()) * node.offset).cast<std::size_t>().eval();

                float& error = node.mip_errors[j];
                for (std::size_t x = 0; x < subregion_size; ++x)
                {
                    for (std::size_t y = 0; y < subregion_size; ++y)
                    {
                        auto p = gi(subregion_offset.x() + x, subregion_offset.y() + y);
                        auto po = original_gi(subregion_offset.x() + x, subregion_offset.y() + y);
                        error = std::max(error, (p - po).norm());

                        node.bounding_box_start = node.bounding_box_start.cwiseMin(po).eval();
                        node.bounding_box_end = node.bounding_box_end.cwiseMax(po).eval();
                    }
                }
            }
        }
        else
        {
            for (std::size_t j = 0; j < 4; ++j)
            {
                auto& child = nodes[child_of(idx, j)];
                for (std::size_t k = 0; k < nodes[idx].mip_errors.size(); ++k)
                {
                    node.mip_errors[k] = std::max(node.mip_errors[k], child.mip_errors[k]);
                }
                node.bounding_box_start = node.bounding_box_start.cwiseMin(child.bounding_box_start).eval();
                node.bounding_box_end = node.bounding_box_end.cwiseMax(child.bounding_box_end).eval();
            }
        }
    }
}

NodeIdx AtlasPatch::child_of(NodeIdx idx, std::size_t child)
{
    return 4*idx + 1 + child;
}

NodeIdx AtlasPatch::parent_of(NodeIdx idx)
{
    return (idx - 1)/4;
}

bool AtlasPatch::has_parent(NodeIdx idx)
{
    return idx != 0;
}

bool AtlasPatch::has_children(NodeIdx idx) const
{
    return 4*idx + 1 < nodes.size();
}
