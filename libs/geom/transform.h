#ifndef GEOM_TRANSFORM_HEADER
#define GEOM_TRANSFORM_HEADER

math::Matrix4f inverse_transform(math::Matrix4f const & T) {
    math::Matrix3f R;
    R(0, 0) = T(0, 0); R(0, 1) = T(1, 0); R(0, 2) = T(2, 0);
    R(1, 0) = T(0, 1); R(1, 1) = T(1, 1); R(1, 2) = T(2, 1);
    R(2, 0) = T(0, 2); R(2, 1) = T(1, 2); R(2, 2) = T(2, 2);
    math::Matrix3f S(0.0f);
    S(0, 0) = 1.0f / R.row(0).norm();
    S(1, 1) = 1.0f / R.row(1).norm();
    S(2, 2) = 1.0f / R.row(2).norm();
    R = S * R;
    math::Vec3f t = -R * S * math::Vec3f(T(0, 3), T(1, 3), T(2, 3));
    return (R * S).hstack(t).vstack(math::Vec4f(0.0f, 0.0f, 0.0f, 1.0f));
}

#endif /* GEOM_TRANSFORM_HEADER */
