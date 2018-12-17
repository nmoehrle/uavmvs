/*
 * Copyright (C) 2016-2018, Nils Moehrle
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include "trajectory.h"

#include "bspline.h"

UTP_NAMESPACE_BEGIN

math::Matrix3f rotation_from_spherical(float theta, float phi) {;
    float ctheta = std::cos(theta);
    float stheta = std::sin(theta);
    float cphi = std::cos(phi);
    float sphi = std::sin(phi);
    math::Vec3f view_dir(stheta * cphi, stheta * sphi, ctheta);
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

    float theta = std::acos(view_dir[2]);
    float phi = std::atan2(view_dir[1], view_dir[0]);

    return std::make_pair(theta, phi);
}

float length(Trajectory const & trajectory) {
    std::vector<math::Vec3f> poss(trajectory.size());
    for (std::size_t i = 0; i < trajectory.size(); ++i) {
        trajectory[i].fill_camera_pos(poss[i].begin());
    }

    utp::BSpline<float, 3u, 3u> spline;
    spline.fit(poss);

    float ret = 0.0f;

    math::Vec3f last = spline.eval(0.0f);
    for (std::size_t i = 1; i < trajectory.size() * 1000; ++i) {
        math::Vec3f curr = spline.eval(i / (trajectory.size() * 1000.0f - 1.0f));
        ret += (curr - last).norm();
        last = curr;
    }

    return ret;
}

UTP_NAMESPACE_END
