/*
 * Copyright (C) 2016-2018, Nils Moehrle
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include <iostream>

#include "math/matrix_tools.h"

#include "util/file_system.h"
#include "util/arguments.h"

#include "util/matrix_io.h"

#include "mve/scene.h"

#include "geom/transform.h"

typedef unsigned int uint;

struct Arguments {
    std::string in_scene;
    std::string out_scene;
    std::string transform;
    bool invert;
};

Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_maxnum(2);
    args.set_nonopt_minnum(2);
    args.set_usage("Usage: " + std::string(argv[0]) + " [OPTS] IN_SCENE OUT_SCENE");
    args.set_description("Conversion app for scenes a la image magicks convert");
    args.add_option('t', "transform", true, "transform vertices with matrix file");
    args.add_option('i', "invert", false, "invert transform");
    args.parse(argc, argv);

    Arguments conf;
    conf.in_scene = args.get_nth_nonopt(0);
    conf.out_scene = args.get_nth_nonopt(1);
    conf.invert = false;

    for (util::ArgResult const* i = args.next_option();
         i != nullptr; i = args.next_option()) {
        switch (i->opt->sopt) {
        case 't':
            conf.transform = i->arg;
        break;
        case 'i':
            conf.invert = true;
        break;
        default:
            throw std::invalid_argument("Invalid option");
        }
    }

    if (conf.in_scene != conf.out_scene) {
        throw std::runtime_error("Copying of scenes is not implemented");
    }

    return conf;
}

void
transform(mve::CameraInfo *camera, math::Matrix4f const & T, math::Matrix3f const & R) {
    math::Matrix3f rot(camera->rot);
    math::Vec3f trans(camera->trans);
    math::Vec3f pos = -rot.transposed() * trans;

    pos = T.mult(pos, 1.0f);
    rot = rot * R.transposed();
    trans = -rot * pos;

    std::copy(trans.begin(), trans.begin() + 3, camera->trans);
    std::copy(rot.begin(), rot.begin() + 9, camera->rot);
}

int main(int argc, char **argv) {
    Arguments args = parse_args(argc, argv);

    mve::Scene::Ptr scene;
    try {
        scene = mve::Scene::create(args.in_scene);
    } catch (std::exception& e) {
        std::cerr << "Could not open scene: " << e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }

    std::string bundle_file;
    bundle_file = util::fs::join_path(scene->get_path(), MVE_SCENE_BUNDLE_FILE);

    mve::Bundle::ConstPtr bundle;
    if (util::fs::exists(bundle_file.c_str())) {
        bundle = scene->get_bundle();
    }

    if (!args.transform.empty()) {
        math::Matrix4f T = load_matrix_from_file<float, 4, 4>(args.transform);

        if (args.invert) {
            T = math::matrix_invert_trans(T);
        }

        math::Matrix3f R;
        float s;
        std::tie(R, s, std::ignore) = split_transform(T);

        if (bundle != nullptr) {
            mve::Bundle::Ptr obundle = mve::Bundle::create();

            std::vector<mve::Bundle::Feature3D> const & features = bundle->get_features();
            std::vector<mve::Bundle::Feature3D> & ofeatures = obundle->get_features();
            ofeatures = features;
            for (mve::Bundle::Feature3D & feature : ofeatures) {
                math::Vec3f pos = T.mult(math::Vec3f(feature.pos), 1.0f);
                std::copy(pos.begin(), pos.begin() + 3, feature.pos);
            }

            std::vector<mve::CameraInfo> const & cameras = bundle->get_cameras();
            std::vector<mve::CameraInfo> & ocameras = obundle->get_cameras();
            ocameras = cameras;
            for (mve::CameraInfo & camera : ocameras) {
                transform(&camera, T, R);
            }

            scene->set_bundle(obundle);
        }

        std::vector<mve::View::Ptr> views = scene->get_views();
        for (mve::View::Ptr & view : views) {
            if (view == nullptr) continue;

            mve::CameraInfo camera = view->get_camera();
            transform(&camera, T, R);
            view->set_camera(camera);

            /* Scale all depth images. */
            std::vector<mve::View::ImageProxy> const & proxys = view->get_images();
            for (mve::View::ImageProxy const & proxy : proxys) {
                if (proxy.name.compare(0, 5, "depth")) continue;

                mve::FloatImage::Ptr image = view->get_float_image(proxy.name);
                for (float * it = image->begin(); it < image->end(); ++it) {
                    *it *= s;
                }
                view->set_image(image, proxy.name);
            }
            view->save_view();
        }
    }

    scene->save_scene();
}
