#ifndef UTP_TRAJECTORY_HEADER
#define UTP_TRAJECTORY_HEADER

#include "mve/camera.h"

#include "defines.h"

UTP_NAMESPACE_BEGIN

typedef std::vector<mve::CameraInfo> Trajectory;

math::Matrix3f rotation_from_spherical(float theta, float phi) {
    float ctheta = std::cos(theta);
    float stheta = std::sin(theta);
    float cphi = std::cos(phi);
    float sphi = std::sin(phi);
    math::Vec3f view_dir(ctheta * sphi, stheta * sphi, cphi);
    view_dir.normalize();

    math::Vec3f rz = view_dir;

    math::Vec3f up = math::Vec3f(0.0f, 0.0f, -1.0f);
    bool stable = abs(up.dot(rz)) < 0.99f;
    up = stable ? up : math::Vec3f(cphi, sphi, 0.0f);

    math::Vec3f rx = up.cross(rz).normalize();
    math::Vec3f ry = rz.cross(rx).normalize();

    math::Matrix3f rot;
    for (int k = 0; k < 3; ++k) {
        rot[k] = rx[k];
        rot[3 + k] = ry[k];
        rot[6 + k] = rz[k];
    }
    return rot;
}

std::pair<float, float> spherical_from_rotation(math::Matrix3f const & rot) {
    math::Vec3f view_dir;
    for (int i = 0; i < 3; ++i) {
        view_dir[i] = rot[6 + i];
    }

    view_dir.normalize();

    float theta = std::atan2(view_dir[1], view_dir[0]);
    float phi = std::acos(view_dir[2]);

    return std::make_pair(theta, phi);
}

UTP_NAMESPACE_END

#endif /* UTP_TRAJECTORY_HEADER */
