/*
 * Copyright (C) 2016-2018, Nils Moehrle
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include <cstring>
#include <cassert>
#include <fstream>
#include <iostream>
#include <random>

#include <unistd.h>

#include "mve/scene.h"
#include "mve/bundle.h"
#include "mve/bundle_io.h"

#include "util/system.h"
#include "util/ini_parser.h"
#include "util/arguments.h"
#include "util/file_system.h"

struct Arguments {
    std::string in_scene;
    std::string out_scene;
    std::uint32_t seed;
    float fraction;
};

Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_maxnum(2);
    args.set_nonopt_minnum(2);
    args.set_usage("Usage: " + std::string(argv[0]) + " [OPTS] IN_SCENE OUT_SCENE");
    args.set_description("Reduces a scene by randomly removing a fraction of "
        "the views along with the features that could not have been "
        "reconstructed otherwise.");
    args.add_option('\0', "seed", true, "seed for RNG [0]");
    args.add_option('f', "fraction", true, "fraction of remaining views [0.5f]");
    args.parse(argc, argv);

    Arguments conf;
    conf.in_scene = args.get_nth_nonopt(0);
    conf.out_scene = args.get_nth_nonopt(1);
    conf.fraction = 0.75f;

    for (util::ArgResult const* i = args.next_option();
         i != 0; i = args.next_option()) {
        switch (i->opt->sopt) {
        case 'f':
            conf.fraction = i->get_arg<float>();
        break;
        case '\0':
            if (i->opt->lopt == "seed") {
                conf.seed = i->get_arg<std::uint32_t>();
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

int main(int argc, char **argv) {
    util::system::register_segfault_handler();
    util::system::print_build_timestamp(argv[0]);

    Arguments args = parse_args(argc, argv);

    mve::Scene::Ptr scene;
    try {
        scene = mve::Scene::create(args.in_scene);
    } catch (std::exception& e) {
        std::cerr << "Could not open scene: " << e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }

    mve::Scene::ViewList const & views = scene->get_views();

    std::vector<int> view_ids;
    view_ids.reserve(views.size());

    for (std::size_t i = 0; i < views.size(); ++i) {
        mve::View::Ptr const & view = views[i];
        if (view == nullptr) continue;

        if (view->get_id() != static_cast<int>(i)) {
            std::cerr << "Scene corrupt - view id and idx mismatch" << std::endl;
            std::exit(EXIT_FAILURE);
        }

        view_ids.push_back(i);
    }

    std::default_random_engine gen(args.seed);
    std::shuffle(view_ids.begin(), view_ids.end(), gen);
    std::vector<std::size_t> new_view_ids(view_ids.begin(),
        view_ids.begin() + view_ids.size() * args.fraction);
    std::cout << new_view_ids.size() << std::endl;

    util::fs::mkdir(args.out_scene.c_str());
    std::string views_dir = util::fs::join_path(args.out_scene, "views");
    util::fs::mkdir(views_dir.c_str());

    mve::Bundle::Ptr bundle = mve::Bundle::create();
    try {
        *bundle = *scene->get_bundle();
    } catch (std::exception& e) {
        std::cerr << "Could not load bundle: " << e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }

    /* Copy views and delete now nonexisting cameras from bundle. */
    std::sort(new_view_ids.begin(), new_view_ids.end());
    std::vector<std::size_t>::const_iterator it = new_view_ids.begin();
    for (std::size_t i = 0; i < views.size(); ++i) {
        if (i == *it) {
            mve::View::Ptr const & view = views[i];
            std::string view_dir = util::fs::join_path(views_dir,
                util::fs::basename(view->get_directory()));
            util::fs::mkdir(view_dir.c_str());

            /* Copy metadata file. */
            std::string meta_fname = util::fs::join_path(view_dir, "meta.ini");
            std::ofstream out(meta_fname.c_str(), std::ios::binary);
            if (!out.good()) {
                std::cerr << "Could not write metadata file: "
                    << std::strerror(errno) << std::endl;
                std::exit(EXIT_FAILURE);
            }

            try {
                util::write_ini(view->get_meta_data().data, out);
                out.close();
            } catch (std::exception & e) {
                out.close();
                util::fs::unlink(meta_fname.c_str());
                std::cerr << "Could not write metadata file: "
                    << e.what() << std::endl;
                std::exit(EXIT_FAILURE);
            }

            std::string iname("undist");
            for (mve::View::ImageProxy proxy : view->get_images()) {
                if (proxy.name.compare(0, iname.size(), iname) == 0) {
                    std::string image = util::fs::join_path(view->get_directory(),
                        proxy.filename);
                    std::string link = util::fs::join_path(view_dir, proxy.filename);
                    //std::cout << link << "\n  -> " << image << std::endl;
                    if (symlink(image.c_str(), link.c_str()) != 0) {
                        std::cerr << "Could not create symlink: "
                            << std::strerror(errno) << std::endl;
                        std::exit(EXIT_FAILURE);
                    }
                }
            }

            it += 1;
        } else {
            bundle->delete_camera(i);
        }
    }

    /* Delete cameras that are soly existing within the bundle. */
    for (std::size_t i = views.size(); i < bundle->get_num_cameras(); ++i) {
        bundle->delete_camera(i);
    }

    /* Remove features that required the now deleted cameras for reconstruction. */
    mve::Bundle::Features & features = bundle->get_features();
    mve::Bundle::Features new_features;
    new_features.reserve(features.size());
    for (std::size_t i = 0; i < features.size(); ++i) {
        mve::Bundle::Feature3D const & feature = features[i];
        if (feature.refs.size() < 2) continue;
        new_features.push_back(feature);
    }
    features.swap(new_features);

    mve::save_mve_bundle(bundle, util::fs::join_path(args.out_scene, "synth_0.out"));

    return EXIT_SUCCESS;
}
