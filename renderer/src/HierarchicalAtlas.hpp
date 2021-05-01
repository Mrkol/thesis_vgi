#pragma once

#include <array>
#include <vector>
#include <vulkan/vulkan.hpp>
#include <Eigen/Dense>


class QuadtreeNode
{
    /**
     * The children are located as follows:
     *  0 1
     *  2 3
     */
    std::array<QuadtreeNode*, 4> children;
    /**
     * Size in parametric space. Equals to 2^-level
     */
    const float size;
    /**
     * Offset in parametric space is coord * size
     */
    const Eigen::Vector2f offset;

    const Eigen::Vector3f bounding_box_start;
    const Eigen::Vector3f bounding_box_end;


};

class OutOfCoreTexture
{

};

class PatchRoot
{
    /**
     * Neighbors are located as follows:
     *     0
     *   3 . 1
     *     2
     */
    std::array<PatchRoot*, 4> neighbors;
    QuadtreeNode root;
    /**
     * 0th is is best resolution
     */
    std::vector<OutOfCoreTexture> geometry_image_lods;
};

class HierarchicalAtlas
{


    std::vector<PatchRoot> patches;
};
