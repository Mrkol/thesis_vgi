#include "Camera.hpp"


Eigen::Matrix4f Camera::view() const
{
    using namespace Eigen;
    return
        ( AngleAxisf(roll_, Vector3f::UnitY())
        * AngleAxisf(pitch_, Vector3f::UnitX())
        * AngleAxisf(-yaw_, Vector3f::UnitZ())
        * Translation3f(position_)
        ).matrix();
}

void Camera::move(const Eigen::Vector3f& direction, float speed)
{
    position_ += Eigen::AngleAxisf(yaw_, Eigen::Vector3f::UnitZ()) * direction.normalized() * speed;
}

void Camera::rotate(float dyaw, float dpitch)
{
    yaw_ += dyaw * 0.001f;
    pitch_ += dpitch * 0.001f;
    pitch_ = std::clamp<float>(pitch_, 0, static_cast<float>(EIGEN_PI));
}
