#ifndef GEOM_TRANSFORM_HEADER
#define GEOM_TRANSFORM_HEADER

#include <tuple>

#include "math/matrix_tools.h"

math::Matrix4f inverse_transform(math::Matrix4f const & T) {
    return math::matrix_invert_trans(T);
}

std::tuple<math::Matrix3f, float, math::Vec3f>
split_transform(math::Matrix4f const & T) {
    math::Matrix3f R;
    R(0, 0) = T(0, 0); R(0, 1) = T(0, 1); R(0, 2) = T(0, 2);
    R(1, 0) = T(1, 0); R(1, 1) = T(1, 1); R(1, 2) = T(1, 2);
    R(2, 0) = T(2, 0); R(2, 1) = T(2, 1); R(2, 2) = T(2, 2);
    float s = std::cbrt(math::matrix_determinant(R));
    R = R / s;
    math::Vec3f t(T(0, 3), T(1, 3), T(2, 3));
    return std::tuple<math::Matrix3f, float, math::Vec3f>(R, s, t);
}

math::Matrix4f
assemble_transform(math::Matrix3f R, float s, math::Vec3f t) {
    return (R * s).hstack(t).vstack(math::Vec4f(0.0f, 0.0f, 0.0f, 1.0f));
}

math::Matrix3f extract_rotation(math::Matrix4f const & T) {
    math::Matrix3f R;
    std::tie(R, std::ignore, std::ignore) = split_transform(T);
    return R;
}

#endif /* GEOM_TRANSFORM_HEADER */
