#pragma once

#include <Eigen/Dense>
#include <utility>


class Camera
{
public:
    [[nodiscard]] Eigen::Matrix4f view() const;

    void move(Eigen::Vector3f direction, float speed);
    void rotate(float dyaw, float dpitch);

    void set_position(Eigen::Vector3f pos) { position = std::move(pos); };

private:
    Eigen::Vector3f position{0, 0, 1};
    float yaw{3*EIGEN_PI/4};
    float pitch{3*EIGEN_PI/4};
    float roll{0};
};
