#include <iostream>

#include "util/system.h"
#include "util/arguments.h"

#include "util/io.h"

#include "utp/bspline.h"
#include "utp/trajectory.h"
#include "utp/trajectory_io.h"

struct Arguments {
    std::string in_trajectory;
    std::string mesh;
};

Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_minnum(2);
    args.set_nonopt_maxnum(2);
    args.set_usage("Usage: " + std::string(argv[0]) + " [OPTS] IN_TRAJECTORY MESH");
    args.set_description("Evaluates feasability and  estimates "
        "the length by fitting a B-Spline through the resulting trajectory. "
        "Optionally estimates the minimal distance to the given mesh.");
    args.parse(argc, argv);

    Arguments conf;
    conf.in_trajectory = args.get_nth_nonopt(0);
    conf.mesh = args.get_nth_nonopt(1);

    for (util::ArgResult const* i = args.next_option();
         i != 0; i = args.next_option()) {
        switch (i->opt->sopt) {
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
    bvh_tree = load_mesh_as_bvh_tree(args.mesh);

    std::vector<mve::CameraInfo> trajectory;
    utp::load_trajectory(args.in_trajectory, &trajectory);

    std::vector<math::Vec3f> pos(trajectory.size());
    for (std::size_t i = 0; i < trajectory.size(); ++i) {
        trajectory[i].fill_camera_pos(pos[i].begin());
    }

    std::cout << "Fitting Spline... " << std::flush;
    utp::BSpline<float, 3u, 3u> spline;
    spline.fit(pos);
    std::cout << "done." << std::endl;

    float length = 0.0f;
    float min_dist = std::numeric_limits<float>::max();

    std::cout << "Evaluating spline... " << std::flush;
    math::Vec3f last = spline.eval(0.0f);
    for (std::size_t i = 1; i < trajectory.size() * 1000; ++i) {
        float t = i / (trajectory.size() * 1000.0f - 1.0f);
        math::Vec3f curr = spline.eval(t);

        if (i % 10 == 0) {
            math::Vec3f cp = bvh_tree->closest_point(curr);
            float dist = (curr - cp).norm();
            min_dist = std::min(min_dist, dist);
        }

        length += (curr - last).norm();
        last = curr;
    }
    std::cout << "done." << std::endl;

    std::cout
        << "Length: " << length << '\n'
        << "Minimal distance: " << min_dist
        << std::endl;


    return EXIT_SUCCESS;
}
