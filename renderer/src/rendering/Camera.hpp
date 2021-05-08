#pragma once

#include <Eigen/Dense>


class Camera
{
public:
    [[nodiscard]] Eigen::Matrix4f view() const;

    void move(Eigen::Vector3f direction);
    void rotate(float dyaw, float dpitch);

private:
    Eigen::Vector3f position{0, 0, 1};
    float yaw{3*EIGEN_PI/4};
    float pitch{3*EIGEN_PI/4};
    float roll{0};
};
