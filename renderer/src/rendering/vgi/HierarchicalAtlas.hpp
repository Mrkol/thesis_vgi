#pragma once

#include <array>
#include <vector>
#include <set>
#include <unordered_set>
#include <filesystem>

#include <vulkan/vulkan.hpp>
#include <Eigen/Dense>

#include "GeometryImage.hpp"
#include "AtlasPatch.hpp"



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
    };

    std::vector<std::pair<NodeHandle, CutElement>> dump() const;
    std::vector<NodeHandle> get_nodes() const;
    std::size_t size() const { return elements.size(); }

    void split(NodeHandle node);
    void set_mip(NodeHandle node, std::size_t mip);

private:
    std::vector<NodeHandle> find_side_neighbors(const NodeHandle node, std::size_t side);
    void recalculate_side_mips(NodeHandle node, std::size_t node_side, std::size_t neighbor_side);

private:
    std::unordered_map<NodeHandle, CutElement> elements;
};

class HierarchicalAtlas
{
public:
    explicit HierarchicalAtlas(const std::filesystem::path& images_folder);

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
