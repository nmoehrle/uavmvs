/*
 * Copyright (C) 2016-2018, Nils Moehrle
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include <random>
#include <limits>
#include <iostream>

#include "util/system.h"
#include "util/arguments.h"

#include "util/matrix_io.h"

#include "math/quaternion.h"
#include "math/functions.h"

#include "geom/transform.h"

#include "utp/bspline.h"
#include "utp/trajectory.h"
#include "utp/trajectory_io.h"

struct Arguments {
    std::string in_traj;
    std::string transform;
    std::string out_csv;
    std::string out_traj;
    std::string out_seq;
    float resolution;
    bool invert;
};

Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_minnum(2);
    args.set_nonopt_maxnum(2);
    args.set_usage("Usage: " + std::string(argv[0]) + " [OPTS] IN_TRAJECTORY OUT_CSV");
    args.set_description("Interpolate and write out trajectory csv files.");
    args.add_option('t', "transform", true, "transform vertices with matrix file");
    args.add_option('r', "resolution", true,
        "resolution for the trajectory (waypoints per meter) [1.0]");
    args.add_option('i', "invert", false, "invert transform");
    args.add_option('\0', "trajectory", true, "Write out a native trajectory file []");
    args.add_option('\0', "sequence", true, "Write out a sequence file []");
    args.parse(argc, argv);

    Arguments conf;
    conf.in_traj = args.get_nth_nonopt(0);
    conf.out_csv = args.get_nth_nonopt(1);
    conf.resolution = 1.0f;
    conf.invert = false;

    for (util::ArgResult const* i = args.next_option();
         i != 0; i = args.next_option()) {
        switch (i->opt->sopt) {
        case 't':
            conf.transform = i->arg;
        break;
        case 'r':
            conf.resolution = i->get_arg<float>();
        break;
        case 'i':
            conf.invert = true;
        break;
        case '\0':
            if (i->opt->lopt == "trajectory") {
                conf.out_traj = i->arg;
            } else if (i->opt->lopt == "sequence") {
                conf.out_seq = i->arg;
            } else {
                throw std::invalid_argument("Invalid option");
            }
        break;
        default:
            throw std::invalid_argument("Invalid option");
        }
    }

    return conf;
}

//TODO move into mve
template <typename T>
math::Quaternion<T>
rot2quat(math::Matrix<T, 3, 3> const & R) {
    T v0 = 1.0 + R(0, 0) + R(1, 1) + R(2, 2);
    T v1 = 1.0 + R(0, 0) - R(1, 1) - R(2, 2);
    T v2 = 1.0 - R(0, 0) + R(1, 1) - R(2, 2);
    T v3 = 1.0 - R(0, 0) - R(1, 1) + R(2, 2);

    //TODO handle nan

    math::Quaternion<T> q;

    if (v0 >= v1 && v0 >= v2 && v0 >= v3) {
        T tmp = T(0.5) * std::sqrt(v0);
        q[0] = tmp;
        q[1] = (R(2, 1) - R(1, 2)) / (T(4.0) * tmp);
        q[2] = (R(0, 2) - R(2, 0)) / (T(4.0) * tmp);
        q[3] = (R(1, 0) - R(0, 1)) / (T(4.0) * tmp);

        return q;
    }

    if (v1 >= v0 && v1 >= v2 && v1 >= v3) {
        T tmp = T(0.5) * std::sqrt(v1);
        q[0] = (R(2, 1) - R(1, 2)) / (T(4.0) * tmp);
        q[1] = tmp;
        q[2] = (R(0, 1) + R(1, 0)) / (T(4.0) * tmp);
        q[3] = (R(0, 2) + R(2, 0)) / (T(4.0) * tmp);

        return q;
    }

    if (v2 >= v0 && v2 >= v1 && v2 >= v3) {
        T tmp = T(0.5) * std::sqrt(v2);
        q[0] = (R(0, 2) - R(2, 0)) / (T(4.0) * tmp);
        q[1] = (R(0, 1) + R(1, 0)) / (T(4.0) * tmp);
        q[2] = tmp;
        q[3] = (R(1, 2) + R(2, 1)) / (T(4.0) * tmp);

        return q;
    }

    //if (v3 >= v0 && v3 >= v1 && v3 >= v2) {
        T tmp = T(0.5) * std::sqrt(v3);
        q[0] = (R(1, 0) - R(0, 1)) / (T(4.0) * tmp);
        q[1] = (R(0, 2) + R(2, 0)) / (T(4.0) * tmp);
        q[2] = (R(1, 2) + R(2, 1)) / (T(4.0) * tmp);
        q[3] = tmp;

        return q;
    //}
}

#define SQR(x) ((x) * (x))
template <typename T>
std::tuple<T, T, T> quat2euler(math::Quaternion<T> q) {
    T phi = std::atan2(T(2) * (q[0] * q[1] + q[2] * q[3]),
        1 - T(2) * (SQR(q[1]) + SQR(q[2])));
    T tmp = T(2) * (q[0] * q[2] - q[3] * q[1]);
    T theta = std::asin(math::clamp(tmp, T(-1.0), T(1.0)));
    T psi = std::atan2(T(2) * (q[0] * q[3] + q[1] * q[2]),
        1 - T(2) * (SQR(q[2]) + SQR(q[3])));
    return std::make_tuple(phi, theta, psi);
}

//TODO move into mve
template <typename T>
math::Vector<T, 4> slerp(math::Vector<T, 4> const & v0,
    math::Vector<T, 4> v1, T t)
{
    T omega = v0.dot(v1);

    if (omega > T(0.9995)) {
        math::Vector<T, 4> ret = v0 + t * (v1 - v0);
        ret.normalize();
        return ret;
    }

    if (omega < T(0.0)) {
        v1 = -v1;
        omega = -omega;
        omega = omega < T(1.0) ? omega : T(1.0);
    }

    T theta = std::acos(omega) * t;

    math::Vector<T, 4> v2 = v1 - v0 * omega;
    v2.normalize();

    return v0 * std::cos(theta) + v2 * sin(theta);
}

int main(int argc, char **argv) {
    util::system::register_segfault_handler();
    util::system::print_build_timestamp(argv[0]);

    Arguments args = parse_args(argc, argv);

    std::vector<mve::CameraInfo> trajectory;
    utp::load_trajectory(args.in_traj, &trajectory);

    std::vector<math::Vec3f> pos(trajectory.size());
    for (std::size_t i = 0; i < trajectory.size(); ++i) {
        trajectory[i].fill_camera_pos(pos[i].begin());
    }

    std::cout << "Fitting Spline... " << std::flush;
    utp::BSpline<float, 3u, 3u> spline;
    spline.fit(pos);
    std::cout << "done." << std::endl;

    if (trajectory.empty()) return EXIT_SUCCESS;

    std::vector<math::Vec3d> xs;
    std::vector<math::Quat4f> qs;
    std::vector<bool> keys;

    std::vector<float> ts = utp::BSpline<float, 3u, 3u>::generate_ts(pos);
    for (std::size_t i = 1; i < trajectory.size(); ++i) {
        float s = ts[i - 1];
        float e = ts[i - 0];

        math::Vec3f last;

        /* Estimate segment length. */
        float length = 0.0f;
        last = spline.eval(s);
        for (float t = s; t <= e; t += (e - s) / 1000.0f) {
            math::Vec3f curr = spline.eval(t);
            length += (curr - last).norm();

            last = curr;
        }

        /* Determine spacing. */
        int n = std::ceil(length / args.resolution);
        float spacing = length / n;

        math::Quat4f sq = rot2quat(math::Matrix3f(trajectory[i - 1].rot));
        math::Quat4f eq = rot2quat(math::Matrix3f(trajectory[i].rot));

#if DEBUG_ANGLES
        float phi, theta, psi;
        std::tie(phi, theta, psi) = quat2euler(eq.conjugated() * math::Quat4f(0, 1, 0, 0));
        float roll = MATH_RAD2DEG(phi);
        float pitch = MATH_RAD2DEG(theta);
        float yaw = MATH_RAD2DEG(psi);
        yaw = yaw < 0.0f ? yaw * -1 : 360 - yaw;
        std::cout << i << ' ' << roll << ' ' << pitch << ' ' << yaw << std::endl;
#endif

        float t = s;
        float clength = 0.0f;
        last = spline.eval(s);
        for (int j = 0; j < n; ++j) {
            while (clength < j * spacing) {
                t += (e - s) / 1000.0f;
                math::Vec3f curr = spline.eval(t);
                clength += (curr - last).norm();
                last = curr;
            }

            xs.push_back(last); //TODO lerp

            math::Vec4f q = slerp(sq, eq, static_cast<float>(j) / n).normalize();
            qs.emplace_back(q[0], q[1], q[2], q[3]);

            keys.push_back(j == 0);
        }

        /* Add last key position. */
        if (i == trajectory.size() - 1) {
            xs.push_back(spline.eval(e));
            qs.push_back(eq);
            keys.push_back(true);
        }
    }

    if (!args.transform.empty()) {
        math::Matrix4d T = load_matrix_from_file<double, 4, 4>(args.transform);

        if (args.invert) {
            T = math::matrix_invert_trans(T);
        }

        math::Quat4f q = rot2quat(math::Matrix3f(extract_rotation(T)));

        for (std::size_t i = 0; i < xs.size(); ++i) {
            xs[i] = T.mult(xs[i], 1.0f);

            qs[i] *= q;
            qs[i].normalize();
        }
    }

    std::ofstream out(args.out_csv.c_str());
    if (!out.good()) {
        std::cerr << "Could not open file" << std::endl;
        std::exit(EXIT_FAILURE);
    }

    out << "x,y,z,qw,qx,qy,qz,key" << std::endl;
    for (std::size_t i = 0; i < xs.size(); ++i) {
        math::Vec3d x = xs[i];
        math::Quat4f q = qs[i];

        out << std::fixed
            << x[0] << ',' << x[1] << ',' << x[2] << ','
            << std::defaultfloat
            << q[0] << ',' << q[1] << ',' << q[2] << ',' << q[3] << ','
            << keys[i] << std::endl;
    }
    out.close();


    if (!args.out_traj.empty()) {
        mve::CameraInfo cam = trajectory[0];
        std::vector<mve::CameraInfo> traj;
        for (std::size_t i = 0; i < xs.size(); ++i) {
            math::Vec3f x = xs[i];
            math::Quat4f q = qs[i];

            math::Matrix3f R;
            math::Vec3f t;

            q.to_rotation_matrix(R.begin());
            t = -R * x;

            std::copy(t.begin(), t.begin() + 3, cam.trans);
            std::copy(R.begin(), R.begin() + 9, cam.rot);
            traj.push_back(cam);
        }
        utp::save_trajectory(traj, args.out_traj);
    }

    if (!args.out_seq.empty()) {
        std::vector<mve::CameraInfo> traj;

        std::ofstream out(args.out_seq.c_str());
        if (!out.good()) {
            std::cerr << "Could not open file" << std::endl;
            std::exit(EXIT_FAILURE);
        }

        int length = static_cast<int>(xs.size());
        out << "fps 30" << '\n';
        out << "sequence trajectory" << '\n';
        out << "length " << xs.size() * 100 << '\n';
        out << "camera-spline-begin" << '\n';
        for (int i = -2; i < length + 2; ++i) {
            std::size_t idx = std::max(0, std::min(i, length - 1));
            math::Vec3f x = xs[idx];
            out << x << '\n';
        }
        out << "camera-spline-end" << '\n';
        out << "lookat-spline-begin" << '\n';
        for (int i = -2; i < length + 2; ++i) {
            std::size_t idx = std::max(0, std::min(i, length - 1));
            math::Vec3f x = xs[idx];
            math::Quat4f q = qs[idx].conjugated();
            out << x + q.rotate(math::Vec3f(0, 0, 50.0f)) << '\n';
        }
        out << "lookat-spline-end" << '\n';
        out.close();
    }

    return EXIT_SUCCESS;
}
