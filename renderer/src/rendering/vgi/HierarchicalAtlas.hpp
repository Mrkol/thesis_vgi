#pragma once

#include <array>
#include <vector>
#include <set>
#include <unordered_set>
#include <filesystem>

#include <vulkan/vulkan.hpp>
#include <Eigen/Dense>


static_assert(sizeof(Eigen::Vector3f) == 3*sizeof(float));

/**
 * Only squares supported
 */
class GeometryImage
{
public:
    GeometryImage() = default;

    explicit GeometryImage(const std::filesystem::path& path);

    [[nodiscard]] GeometryImage upscale(std::size_t newsize) const;

    Eigen::Vector3f& operator() (std::size_t x, std::size_t y);
    const Eigen::Vector3f& operator() (std::size_t x, std::size_t y) const;
    [[nodiscard]] std::size_t size() const { return  width; }

    [[nodiscard]] const Eigen::Vector3f* get_data() const { return data.data(); }

private:
    std::vector<Eigen::Vector3f> data;
    std::size_t width{};
};


// TODO: Rewrite this with an array and indices instead of pointers -- it'll be faster
class QuadtreeNode
{
public:
    struct CreateInfo
    {
        std::size_t levels_left;
        float size;
        Eigen::Vector2f offset;
        const std::vector<GeometryImage>& GIs;
        const std::vector<GeometryImage>& upsampled_GIs;
    };
    explicit QuadtreeNode(CreateInfo info);




    /**
     * Needed to traverse the tree when finding "neighbors" in the cut
     */
    QuadtreeNode* parent{nullptr};
    /**
     * The children are located as follows:
     *  0 1
     *  2 3
     */
    std::array<std::unique_ptr<QuadtreeNode>, 4> children;
    /**
     * Size in parametric space. Equals to 2^-level
     */
    float size;
    /**
     * Offset in parametric space is coord * size
     */
    Eigen::Vector2f offset;

    Eigen::Vector3f bounding_box_start;
    Eigen::Vector3f bounding_box_end;

    // When a node covers a small amount of param space it shouldn't be able to over or under sample our GIs with
    // too high or too low tesselation
    std::size_t min_tessellation;
    std::size_t max_tessellation;
    /**
     * Geometric errors for each geometry image mip level
     */
    std::vector<float> mip_errors;
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

    explicit AtlasPatch(CreateInfo info);

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

    std::unique_ptr<QuadtreeNode> root;

    std::vector<GeometryImage> gis;
};

class HierarchyCut
{
    friend class HierarchicalAtlas;
public:
    HierarchyCut() = default;

    struct CutElement
    {
        std::size_t patch_idx;
        std::size_t mip;
        std::array<std::size_t, 4> side_mip;
        AtlasPatch* root;
    };

    std::vector<std::pair<QuadtreeNode*, CutElement>> dump() const;
    const CutElement& get_data(QuadtreeNode* node) const;
    std::size_t size() const { return elements.size(); }

    void split(QuadtreeNode* node);
    void set_mip(QuadtreeNode* node, std::size_t mip);

private:
    std::vector<QuadtreeNode*> find_side_neighbors(QuadtreeNode* node, std::size_t side);
    void recalculate_side_mips(QuadtreeNode* node, std::size_t node_side, std::size_t neighbor_side);

private:
    std::unordered_map<QuadtreeNode*, CutElement> elements;
};

class HierarchicalAtlas
{
public:
    explicit HierarchicalAtlas(std::filesystem::path images_folder);

    const std::vector<GeometryImage>& get_gis(std::size_t idx) { return patches.at(idx).gis; }

    [[nodiscard]] HierarchyCut default_cut();

    [[nodiscard]] std::size_t get_hierarchy_depth() const { return min_mip; }
    [[nodiscard]] std::size_t get_patch_count() const { return patches.size(); }

    [[nodiscard]] std::size_t get_min_mip() const { return min_mip; }

private:
    std::vector<AtlasPatch> patches;
    std::size_t max_mip;
    std::size_t min_mip;
};
