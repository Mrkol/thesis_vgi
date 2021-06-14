#pragma once

#include <array>
#include <vector>
#include <filesystem>
#include <Eigen/Dense>
#include "GeometryImage.hpp"
#include "TupleHash.hpp"


using NodeIdx = std::size_t;

struct AtlasNode;
class AtlasPatch;

class NodeHandle
{
    friend struct std::hash<NodeHandle>;
public:
    NodeHandle() = default;

    NodeHandle(NodeIdx idx, AtlasPatch* patch) : idx_{idx}, patch_{patch} {};


    [[nodiscard]] static NodeHandle root_of(AtlasPatch* patch) { return NodeHandle{0, patch}; }

    [[nodiscard]] bool valid() const { return patch_ != nullptr; }

    [[nodiscard]] AtlasNode* get() { return valid() ? &operator*() : nullptr; }
    [[nodiscard]] const AtlasNode* get() const { return valid() ? &operator*() : nullptr; }

    [[nodiscard]] AtlasPatch& patch() { return *patch_; }
    [[nodiscard]] const AtlasPatch& patch() const { return *patch_; }


    [[nodiscard]] NodeHandle child(std::size_t child) const;
    [[nodiscard]] bool has_children() const;

    [[nodiscard]] NodeHandle parent() const;
    [[nodiscard]] bool has_parent() const;

    [[nodiscard]] AtlasNode& operator*();
    [[nodiscard]] const AtlasNode& operator*() const;

    [[nodiscard]] AtlasNode* operator->() { return get(); }
    [[nodiscard]] const AtlasNode* operator->() const { return get(); }

    [[nodiscard]] std::weak_ordering operator<=>(const NodeHandle&) const = default;

private:
    NodeIdx idx_{0};
    AtlasPatch* patch_{nullptr};
};

namespace std
{
template<>
struct hash<NodeHandle>
{
    std::size_t operator()(const NodeHandle& handle) const
    {
        return std::hash<std::tuple<std::size_t, AtlasPatch*>>{}({handle.idx_, handle.patch_});
    }
};
}


struct AtlasNode
{
    [[nodiscard]] float projected_screenspace_area(const Eigen::Matrix4f& model, const Eigen::Matrix4f& view, const Eigen::Matrix4f& projection) const;


    /**
     * Size in parametric space. Equals to 2^-level
     */
    float size{1};
    /**
     * Offset in parametric space is coord * size
     */
    Eigen::Vector2f offset{Eigen::Vector2f::Zero()};

    /**
     * When a node covers a small amount of param space it shouldn't be able to over or under sample our GIs with
     * too high or too low tesselation
     */
    std::size_t min_tessellation{0};
    std::size_t max_tessellation{0};

    Eigen::Vector3f bounding_box_start{Eigen::Vector3f::Constant(std::numeric_limits<float>::max())};
    Eigen::Vector3f bounding_box_end{Eigen::Vector3f::Constant(-std::numeric_limits<float>::max())};

    /**
     * Geometric errors for each geometry image mip level
     */
    std::vector<float> mip_errors;

    /**
     * Neighbors are located as follows:
     *     0
     *   3 . 1
     *     2
     */
    std::array<NodeHandle, 4> neighbor_search_starts{};
};

class AtlasPatch
{
public:
    struct CreateInfo
    {
        std::filesystem::path images_folder;
        std::string_view name_prefix;
        std::size_t min_mip;
        std::size_t max_mip;
    };

    explicit AtlasPatch(const CreateInfo& info);

    /**
     * The children are located as follows:
     *  0 1
     *  2 3
     */
    static NodeIdx child_of(NodeIdx idx, std::size_t child);
    static NodeIdx parent_of(NodeIdx idx);
    [[nodiscard]] static bool has_parent(NodeIdx  idx);
    [[nodiscard]] bool has_children(NodeIdx idx) const;

    /**
     * used to reconstruct neighbors
     * 0 1
     * 2 3
     */
    std::array<Eigen::Vector3f, 4> max_res_corners{Eigen::Vector3f::Zero()};

    /**
     * Neighbors are located as follows:
     *     0
     *   3 . 1
     *     2
     */
    std::array<AtlasPatch*, 4> neighbors{nullptr};
    std::array<std::size_t, 4> neighbor_rotation_difference{0};

    /**
     * Stored in quad heap format: (4^(i-1), 4^i] is the ith level
     */
    std::vector<AtlasNode> nodes;

    std::vector<GeometryImage> gis;
};
