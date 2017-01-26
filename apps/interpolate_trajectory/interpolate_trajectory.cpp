#include <random>
#include <limits>
#include <iostream>

#include "util/system.h"
#include "util/arguments.h"

#include "util/io.h"

#include "utp/bspline.h"
#include "utp/trajectory.h"
#include "utp/trajectory_io.h"

#include "tsp/optimize.h"

struct Arguments {
    std::string in_trajectory;
    std::string mesh;
    std::string out_trajectory;
};

Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_minnum(2);
    args.set_nonopt_maxnum(2);
    args.set_usage("Usage: " + std::string(argv[0]) + " [OPTS] IN_TRAJECTORY OUT_TRAJECTORY");
    args.set_description("Searches for a short path trough the input trajectories "
        "view positions by solving the corresponding TSP and estimates "
        "the length by fitting a B-Spline through the resulting trajectory. "
        "Optionally estimates the minimal distance to the given mesh.");
    args.add_option('m', "mesh", true, "mesh for minimal distance calculation");
    args.parse(argc, argv);

    Arguments conf;
    conf.in_trajectory = args.get_nth_nonopt(0);
    conf.out_trajectory = args.get_nth_nonopt(1);

    for (util::ArgResult const* i = args.next_option();
         i != 0; i = args.next_option()) {
        switch (i->opt->sopt) {
        case 'm':
            conf.mesh = i->arg;
        break;
        default:
            throw std::invalid_argument("Invalid option");
        }
    }

    return conf;
}

int main(int argc, char **argv) {
    util::system::register_segfault_handler();
    util::system::print_build_timestamp(argv[0]);

    Arguments args = parse_args(argc, argv);

    acc::BVHTree<uint, math::Vec3f>::Ptr bvh_tree;
    if (!args.mesh.empty()) {
        bvh_tree = load_mesh_as_bvh_tree(args.mesh);
    }

    std::vector<mve::CameraInfo> trajectory;
    utp::load_trajectory(args.in_trajectory, &trajectory);

    std::vector<math::Vec3f> pos(trajectory.size());
    for (std::size_t i = 0; i < trajectory.size(); ++i) {
        trajectory[i].fill_camera_pos(pos[i].begin());
    }
    std::vector<uint> ids(trajectory.size());
    std::iota(ids.begin(), ids.end(), 0);
    std::cout << "Optimizing TSP... " << std::flush;
    tsp::optimize(&ids, pos, 24);
    std::cout << "done. " << std::endl;
    {
        std::vector<mve::CameraInfo> tmp(trajectory.size());
        for (std::size_t i = 0; i < trajectory.size(); ++i) {
            tmp[i] = trajectory[ids[i]];
        }
        std::swap(tmp, trajectory);
    }

    std::vector<math::Vec3f> poss(trajectory.size());
    for (std::size_t i = 0; i < trajectory.size(); ++i) {
        trajectory[i].fill_camera_pos(poss[i].begin());
    }

    std::cout << "Fitting Spline... " << std::flush;
    utp::BSpline<float, 3u, 3u> spline;
    spline.fit(poss);
    std::cout << "done." << std::endl;

    float length = 0.0f;
    float min_dist = std::numeric_limits<float>::max();

    std::cout << "Evaluating spline... " << std::flush;
    math::Vec3f last = spline.eval(0.0f);
    for (std::size_t i = 1; i < trajectory.size() * 1000; ++i) {
        math::Vec3f curr = spline.eval(i / (trajectory.size() * 1000.0f - 1.0f));

        if (i % 100 == 0 && bvh_tree != nullptr) {
            math::Vec3f cp = bvh_tree->closest_point(curr);
            float dist = (curr - cp).norm();
            min_dist = std::min(min_dist, dist);
        }

        length += (curr - last).norm();
        last = curr;
    }
    std::cout << "done." << std::endl;

    std::cout << "Length: " << length << std::endl;
    if (bvh_tree != nullptr) {
        std::cout << "Minimal distance: " << min_dist << std::endl;
    }

    utp::save_trajectory(trajectory, args.out_trajectory);

    return EXIT_SUCCESS;
}
