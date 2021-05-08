#pragma once

#include <Eigen/Dense>

#include "Utility.hpp"


// https://github.com/g-truc/glm/blob/0.9.5/glm/gtc/matrix_transform.inl#L207-L229
template<typename T>
Eigen::Matrix<T, 4, 4> perspective(T fovy, T aspect, T zNear, T zFar)
{
    AD_HOC_ASSERT(aspect > 0, "");
    AD_HOC_ASSERT(zFar > zNear, "");

    auto tanHalfFovy = std::tan(fovy / T{2});
    Eigen::Matrix4f result = Eigen::Matrix4f::Zero();
    result(0,0) = T{1} / (aspect * tanHalfFovy);
    result(1,1) = -T{1} / (tanHalfFovy);
    result(2,2) = -(zFar + zNear) / (zFar - zNear);
    result(3,2) = -T{1};
    result(2,3) = -(T{2} * zFar * zNear) / (zFar - zNear);
    return result;
}
