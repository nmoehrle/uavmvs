/*
 * Copyright (C) 2016-2018, Nils Moehrle
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include <iostream>

#include "util/system.h"
#include "util/arguments.h"
#include "util/tokenizer.h"

#include "math/matrix_tools.h"

#include "util/io.h"

#include "mve/mesh_io_ply.h"

#include "acc/primitives.h"

#include "utp/trajectory_io.h"

struct Arguments {
    std::string proxy_mesh;
    std::string out_trajectory;
    std::string airspace_mesh;
    float forward_overlap;
    float side_overlap;
    float rotation;
    std::vector<float> angles;
    float altitude;
    float elevation;
    float focal_length;
    float aspect_ratio;
};

Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_minnum(2);
    args.set_nonopt_maxnum(2);
    args.set_usage("Usage: " + std::string(argv[0]) + " [OPTS] PROXY_MESH OUT_TRAJECTORY");
    args.set_description("Generate trajectories");
    args.add_option('f', "forward-overlap", true, "forward overlap in percent [80.0]");
    args.add_option('s', "side-overlap", true, "side overlap in percent [60.0]");
    args.add_option('a', "altitude", true, "flying altitude [60.0]");
    args.add_option('e', "elevation", true, "elevation for overlap planning [0.0]");
    args.add_option('r', "rotation", true, "rotation (deg) [0]");

    args.add_option('\0', "angles", true, "comma separate list of angles (nadir 0 deg) [0]");
    args.add_option('\0', "focal-length", true, "camera focal length [0.86]");
    args.add_option('\0', "aspect-ratio", true, "camera sensor aspect ratio [0.66]");
    args.add_option('\0', "airspace-mesh", true, "use mesh to consider flyable airspace");
    args.parse(argc, argv);

    Arguments conf;
    conf.proxy_mesh = args.get_nth_nonopt(0);
    conf.out_trajectory = args.get_nth_nonopt(1);
    conf.forward_overlap = 80.0f;
    conf.side_overlap = 60.0f;
    conf.rotation = 0.0f;
    conf.angles = {0};
    conf.altitude = 60.0f;
    conf.focal_length = 0.86f;
    conf.aspect_ratio = 2.0f / 3.0f;

    std::string angles;
    for (util::ArgResult const* i = args.next_option();
         i != 0; i = args.next_option()) {
        switch (i->opt->sopt) {
        case 'f':
                conf.forward_overlap = i->get_arg<float>();
        break;
        case 's':
            conf.side_overlap = i->get_arg<float>();
        break;
        case 'r':
            conf.rotation = i->get_arg<float>() * M_PI / 180.0f;
        break;
        case 'a':
            conf.altitude = i->get_arg<float>();
        break;
        case 'e':
            conf.elevation = i->get_arg<float>();
        break;
        case '\0':
            if (i->opt->lopt == "angles") {
                angles = i->arg;
            } else if (i->opt->lopt == "focal-length") {
                conf.focal_length = i->get_arg<float>();
            } else if (i->opt->lopt == "aspect-ratio") {
                conf.aspect_ratio = i->get_arg<float>();
            } else if (i->opt->lopt == "airspace-mesh") {
                conf.airspace_mesh = i->arg;
            } else {
                throw std::invalid_argument("Invalid option");
            }
        break;
        default:
            throw std::invalid_argument("Invalid option");
        }
    }

    if (conf.aspect_ratio > 1.0f) {
        throw std::invalid_argument("Aspect ratio has to be less than one");
    }

    if (!angles.empty()) {
        util::Tokenizer tok;
        tok.split(angles, ',');
        conf.angles.clear();

        for (std::size_t i = 0; i < tok.size(); ++i) {
            conf.angles.push_back(tok.get_as<float>(i) * M_PI / 180.0f);
        }
    }

    return conf;
}

int main(int argc, char **argv) {
    util::system::register_segfault_handler();
    util::system::print_build_timestamp(argv[0]);

    Arguments args = parse_args(argc, argv);

    mve::TriangleMesh::Ptr mesh;
    try {
        mesh = mve::geom::load_ply_mesh(args.proxy_mesh);
    } catch (std::exception& e) {
        std::cerr << "\tCould not load mesh: "<< e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }

    acc::BVHTree<uint, math::Vec3f>::Ptr bvh_tree;
    if (!args.airspace_mesh.empty()) {
        bvh_tree = load_mesh_as_bvh_tree(args.airspace_mesh);
    }

    math::Matrix3f rot = math::matrix_rotation_from_axis_angle(
        math::Vec3f(0.0f, 0.0f, 1.0f), args.rotation);

    acc::AABB<math::Vec3f> aabb;
    if (args.rotation == 0.0f) {
        aabb = acc::calculate_aabb(mesh->get_vertices());
    } else {
        std::vector<math::Vec3f> verts = mesh->get_vertices();
        aabb = acc::calculate_aabb(verts);
        math::Vec3f center = aabb.min + (aabb.max - aabb.min) / 2.0f;
        for (std::size_t i = 0; i < verts.size(); ++i) {
            verts[i] = rot * (verts[i] - center) + center;
        }
        aabb = acc::calculate_aabb(verts);
        center = aabb.min + (aabb.max - aabb.min) / 2.0f;
    }


    float hfov = 2.0f * std::atan2(1.0f, 2.0f * args.focal_length);
    float vfov = 2.0f * std::atan2(args.aspect_ratio, 2.0f * args.focal_length);

    /* Compensate for lower parts. */
    float width = std::tan(hfov / 2.0f) * (args.altitude - args.elevation) * 2.0f;
    float height = std::tan(vfov / 2.0f) * (args.altitude - args.elevation) * 2.0f;

    float velocity = height * (1.0f - args.forward_overlap / 100.0f);
    float spacing = width * (1.0f - args.side_overlap / 100.0f);

    math::Vec3f dim = aabb.max - aabb.min;
    math::Vec3f off(aabb.min[0] + dim[0] / 2.0f, aabb.min[1] + dim[1] / 2.0f, args.altitude);

    /* Set camera intrinsics. */
    mve::CameraInfo cam;
    cam.flen = args.focal_length;

    /* Initialize nadir. */
    math::Matrix3f cam_rot(0.0f);
    cam_rot(0, 0) = 1;
    cam_rot(1, 1) = -1;
    cam_rot(2, 2) = -1;

    std::vector<mve::CameraInfo> trajectory;
    math::Vec3f rel(0.0f);
    for (std::size_t d = 0; d < args.angles.size(); ++d) {
        /* Sideways dimension */
        int sw = d % 2;
        /* Forward dimension */
        int fw = (d + 1) % 2;
        int lines = std::max<int>(std::ceil(dim[sw] / spacing), 2);
        int images = std::max<int>(std::ceil(dim[fw] / velocity), 2);

        math::Vec3f axis(0.0f);
        axis[sw] = 1.0f;
        math::Matrix3f rrot = math::matrix_rotation_from_axis_angle(
            axis, args.angles[d]) * rot * cam_rot;
        std::copy(rrot.begin(), rrot.end(), cam.rot);

        /* Determine flight directions based on last relative location. */
        float ss = (0.0f < rel[sw]) ? -1.0f : 1.0f;
        float fs = (0.0f < rel[fw]) ? -1.0f : 1.0f;

        for (int i = 0; i < lines; ++i) {
            rel = math::Vec3f(0.0f);
            rel[sw] = ss * spacing * (i - (lines - 1) / 2.0f);

            /* Determine flight direction for this line. */
            float lfs = fs * ((i % 2 == 0) ? 1.0f : -1.0f);
            for (int j = 0; j < images - 1; ++j) {
                rel[fw] = lfs * velocity * (j - ((images - 1) / 2.0f));

                math::Vec3f pos = off + rot * rel;
                math::Vec3f trans = -rrot * pos;
                std::copy(trans.begin(), trans.end(), cam.trans);
                trajectory.push_back(cam);
            }

            float radius = spacing / 2.0f;

            int n = 1;
            /* If not last line, determine number of connecting images. */
            if (i != (lines - 1)) {
                n = std::floor((M_PI * radius) / velocity);
            }

            /* Connect lines via half circle. */
            float angle = std::acos(-1.0f) / n;
            for (int j = 0; j < n; j++) {
                math::Vec2f r;
                r[sw] = std::cos(angle * j) * radius;
                r[fw] = std::sin(angle * j) * radius;

                rel = math::Vec3f(0.0f);
                rel[sw] = ss * ((i - (lines - 1) / 2.0f) * spacing + radius - r[sw]);
                rel[fw] = lfs * (((images - 1) / 2.0f) * velocity + r[fw]);

                math::Vec3f pos = off + rot * rel;
                math::Vec3f trans = -rrot * pos;
                std::copy(trans.begin(), trans.end(), cam.trans);
                trajectory.push_back(cam);
            }
        }
    }

    std::cout << trajectory.size() << std::endl;

    /* Adjust heights based on airspace mesh. */
    if (bvh_tree != nullptr) {
        for (std::size_t i = 0; i < trajectory.size(); ++i) {
            mve::CameraInfo & cam = trajectory[i];

            math::Vec3f pos;
            cam.fill_camera_pos(pos.begin());

            acc::BVHTree<uint, math::Vec3f>::Ray ray;
            ray.origin = pos;
            ray.dir = math::Vec3f(0.0f, 0.0f, 1.0f);
            ray.tmin = 0.0f;
            ray.tmax = std::numeric_limits<float>::infinity();

            acc::BVHTree<uint, math::Vec3f>::Hit hit;
            if (!bvh_tree->intersect(ray, &hit)) continue;

            pos[2] += hit.t + args.altitude / 20.0f;
            math::Matrix3f cam_rot(cam.rot);
            math::Vec3f trans = -cam_rot * pos;
            std::copy(trans.begin(), trans.end(), cam.trans);
        }
    }

    utp::save_trajectory(trajectory, args.out_trajectory);

    return EXIT_SUCCESS;
}
