#include <iostream>

#include "util/system.h"
#include "util/arguments.h"

#include "utp/bspline.h"
#include "utp/trajectory_io.h"

#include "tsp/optimize.h"

struct Arguments {
    std::string in_trajectory;
    std::string out_trajectory;
};

Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_minnum(2);
    args.set_nonopt_maxnum(2);
    args.set_usage("Usage: " + std::string(argv[0]) + " [OPTS] IN_TRAJECTORY OUT_TRAJECTORY");
    args.set_description("TODO");
    args.parse(argc, argv);

    Arguments conf;
    conf.in_trajectory = args.get_nth_nonopt(0);
    conf.out_trajectory = args.get_nth_nonopt(1);

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

    std::vector<mve::CameraInfo> trajectory;
    utp::load_trajectory(args.in_trajectory, &trajectory);

    std::vector<math::Vec3f> pos(trajectory.size());
    for (std::size_t i = 0; i < trajectory.size(); ++i) {
        trajectory[i].fill_camera_pos(pos[i].begin());
    }
    std::vector<uint> ids(trajectory.size());
    std::iota(ids.begin(), ids.end(), 0);
    tsp::optimize(&ids, pos, 24);
    {
        std::vector<mve::CameraInfo> tmp(trajectory.size());
        for (std::size_t i = 0; i < trajectory.size(); ++i) {
            tmp[i] = trajectory[ids[i]];
        }
        std::swap(tmp, trajectory);
    }
    for (std::size_t i = 0; i < trajectory.size(); ++i) {
        trajectory[i].fill_camera_pos(pos[i].begin());
    }

    utp::BSpline<float, 3u, 3u> spline;
    spline.fit(pos);

    float length = 0.0f;

    math::Vec3f last = spline.eval(0.0f);
    for (std::size_t i = 1; i < trajectory.size() * 100; ++i) {
        math::Vec3f curr = spline.eval(i / (trajectory.size() * 100.0f));
        length += (curr - last).norm();
        last = curr;
    }
    std::cout << "Length: " <<length << std::endl;

    utp::save_trajectory(trajectory, args.out_trajectory);

    return EXIT_SUCCESS;
}
