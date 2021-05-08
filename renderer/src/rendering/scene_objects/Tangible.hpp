#pragma once

#include <Eigen/Geometry>

#include "../SceneObjectBase.hpp"

class TangibleSceneObject : public SceneObjectBase
{
public:
    Eigen::Vector3f position{0, 0, 0};
    Eigen::Vector3f scale{1, 1, 1};
    Eigen::Quaternionf rotation{1, 0, 0, 0};
};
