/*
 * Copyright (C) 2016-2018, Nils Moehrle
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include <iostream>
#include <algorithm>

#include "util/system.h"
#include "util/arguments.h"

#include "mve/scene.h"

struct Arguments {
    std::string in_scene;
    std::string gt_scene;
};

Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_minnum(2);
    args.set_nonopt_maxnum(2);
    args.set_usage("Usage: " + std::string(argv[0]) + " [OPTS] IN_SCENE GT_SCENE");
    args.set_description("Positional error");
    args.parse(argc, argv);

    Arguments conf;
    conf.in_scene = args.get_nth_nonopt(0);
    conf.gt_scene = args.get_nth_nonopt(1);

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

    mve::Scene::Ptr in_scene;
    try {
        in_scene = mve::Scene::create(args.in_scene);
    } catch (std::exception& e) {
        std::cerr << "Could not open scene: " << e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }

    mve::Scene::Ptr gt_scene;
    try {
        gt_scene = mve::Scene::create(args.gt_scene);
    } catch (std::exception& e) {
        std::cerr << "Could not open scene: " << e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }

    std::vector<mve::View::Ptr> const & in_views = in_scene->get_views();
    std::vector<mve::View::Ptr> const & gt_views = gt_scene->get_views();

    if (in_views.size() != gt_views.size()) {
        std::cerr << "Incompatible scenes" << std::endl;
        std::exit(EXIT_FAILURE);
    }

    std::size_t num_views = in_views.size() & gt_views.size();

    std::vector<float> errors;
    errors.reserve(num_views);
    for (std::size_t i = 0; i < num_views; ++i) {
        mve::CameraInfo in_cam = in_views[i]->get_camera();
        if (in_cam.flen == 0.0f) continue;

        mve::CameraInfo gt_cam = gt_views[i]->get_camera();
        math::Vec3f in_pos;
        in_cam.fill_camera_pos(in_pos.begin());
        math::Vec3f gt_pos;
        gt_cam.fill_camera_pos(gt_pos.begin());

        errors.push_back((in_pos - gt_pos).norm());
    }

    if (errors.empty()) {
        std::cerr << "No views reconstructed" << std::endl;
        std::exit(EXIT_FAILURE);
    }

    std::vector<float>::iterator nth = errors.begin() + errors.size() / 2;
    std::nth_element(errors.begin(), nth, errors.end());

    std::cout << *nth << std::endl;

    return EXIT_SUCCESS;
}
