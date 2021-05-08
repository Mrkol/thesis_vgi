#include "Camera.hpp"


Eigen::Matrix4f Camera::view() const
{
    using namespace Eigen;
    return
        ( AngleAxisf(roll, Vector3f::UnitY())
        * AngleAxisf(pitch, Vector3f::UnitX())
        * AngleAxisf(-yaw, Vector3f::UnitZ())
        * Translation3f(position)
        ).matrix();
}

void Camera::move(Eigen::Vector3f direction)
{
    position += Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) * direction.normalized() * 0.001f;
}

void Camera::rotate(float dyaw, float dpitch)
{
    yaw += dyaw * 0.001f;
    pitch += dpitch * 0.001f;
    pitch = std::clamp<float>(pitch, 0, EIGEN_PI);

}
