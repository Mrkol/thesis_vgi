#pragma once

#include <Eigen/Dense>
#include <utility>


class Camera
{
public:
    [[nodiscard]] Eigen::Matrix4f view() const;

    void move(const Eigen::Vector3f& direction, float speed);
    void rotate(float dyaw, float dpitch);

    void set_position(Eigen::Vector3f pos) { position_ = std::move(pos); };

private:
    Eigen::Vector3f position_{0, 0, 1};
    float yaw_{static_cast<float>(3 * EIGEN_PI / 4)};
    float pitch_{static_cast<float>(3 * EIGEN_PI / 4)};
    float roll_{0};
};
