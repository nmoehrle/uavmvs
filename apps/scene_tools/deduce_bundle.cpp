/*
 * Copyright (C) 2016-2018, Nils Moehrle
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include <iostream>

#include "util/timer.h"
#include "util/system.h"
#include "util/arguments.h"
#include "util/file_system.h"

#include "mve/scene.h"
#include "mve/image_tools.h"

#include "sfm/bundler_common.h"
#include "sfm/bundler_features.h"
#include "sfm/bundler_matching.h"
#include "sfm/bundler_tracks.h"
#include "sfm/bundler_incremental.h"

#include "util/helpers.h"

struct Arguments {
    std::string scene;
};

Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_minnum(1);
    args.set_nonopt_maxnum(1);
    args.set_usage("Usage: " + std::string(argv[0]) + " [OPTS] SCENE");
    args.set_description("Creates a bundle by extracting, matching "
        "and triangulating features for a scene.");
    args.parse(argc, argv);

    Arguments conf;
    conf.scene = args.get_nth_nonopt(0);

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

    mve::Scene::Ptr scene;
    try {
        scene = mve::Scene::create(args.scene);
    } catch (std::exception& e) {
        std::cerr << "Could not open scene: " << e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }

    /* Derived from mve/apps/sfmrecon/sfmrecon.cc */

    sfm::bundler::ViewportList viewports;
    sfm::bundler::PairwiseMatching pairwise_matching;

    sfm::bundler::Features::Options feature_opts;
    feature_opts.image_embedding = "original";
    feature_opts.max_image_size = 6000000;
    feature_opts.feature_options.feature_types = sfm::FeatureSet::FEATURE_ALL;

    std::cout << "Computing image features..." << std::endl;
    {
        util::WallTimer timer;
        sfm::bundler::Features bundler_features(feature_opts);
        bundler_features.compute(scene, &viewports);

        std::cout << "Computing features took " << timer.get_elapsed()
            << " ms." << std::endl;
    }

    /* Exhaustive matching between all pairs of views. */
    sfm::bundler::Matching::Options matching_opts;
    matching_opts.ransac_opts.verbose_output = false;
    matching_opts.use_lowres_matching = false;
    matching_opts.match_num_previous_frames = false;
    matching_opts.matcher_type = sfm::bundler::Matching::MATCHER_CASCADE_HASHING;

    std::cout << "Performing feature matching..." << std::endl;
    {
        util::WallTimer timer;
        sfm::bundler::Matching bundler_matching(matching_opts);
        bundler_matching.init(&viewports);
        bundler_matching.compute(&pairwise_matching);
        std::cout << "Matching took " << timer.get_elapsed()
            << " ms." << std::endl;
    }

    sfm::bundler::TrackList tracks;
    sfm::bundler::Tracks::Options tracks_options;
    tracks_options.verbose_output = true;

    sfm::bundler::Tracks bundler_tracks(tracks_options);
    std::cout << "Computing feature tracks..." << std::endl;
    bundler_tracks.compute(pairwise_matching, &viewports, &tracks);
    std::cout << "Created a total of " << tracks.size()
        << " tracks." << std::endl;

    sfm::bundler::Incremental::Options incremental_opts;
    incremental_opts.track_error_threshold_factor = 25.0f;
    incremental_opts.new_track_error_threshold = 0.01f;
    incremental_opts.min_triangulation_angle = MATH_DEG2RAD(1.0);
    incremental_opts.verbose_output = true;

    mve::Scene::ViewList const & views = scene->get_views();
    for (std::size_t i = 0; i < views.size(); ++i) {
        mve::View::Ptr const & view = views[i];
        if (view == nullptr) continue;

        mve::View::ImageProxy const * proxy = view->get_image_proxy("original");
        int width = proxy->width;
        int height = proxy->height;

        mve::CameraInfo cam = view->get_camera();
        sfm::CameraPose * pose = &(viewports[i].pose);

        fill_camera_pose(cam, width, height, pose);
    }

    /* Initialize the incremental bundler and reconstruct first tracks. */
    sfm::bundler::Incremental incremental(incremental_opts);
    incremental.initialize(&viewports, &tracks);
    incremental.triangulate_new_tracks(2);
    incremental.invalidate_large_error_tracks();

    mve::Bundle::Ptr bundle = incremental.create_bundle();

    scene->set_bundle(bundle);
    scene->save_bundle();

    return EXIT_SUCCESS;
}
