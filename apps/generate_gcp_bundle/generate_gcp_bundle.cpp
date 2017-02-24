#include <cerrno>
#include <cstring>
#include <fstream>
#include <iostream>
#include <unordered_map>

#include "util/system.h"
#include "util/arguments.h"

#include "mve/scene.h"
#include "mve/bundle_io.h"
#include "mve/image_tools.h"

#include "sfm/bundler_common.h"
#include "sfm/bundler_incremental.h"

#include "util/helpers.h"

struct Arguments {
    std::string gcpfile;
    std::string scene;
    std::string bundle;
};

Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_minnum(3);
    args.set_nonopt_maxnum(3);
    args.set_usage("Usage: " + std::string(argv[0]) + " [OPTS] GCPFILE SCENE BUNDLE");
    args.set_description("Generates a bundle from gcp csv file.");
    args.parse(argc, argv);

    Arguments conf;
    conf.gcpfile = args.get_nth_nonopt(0);
    conf.scene = args.get_nth_nonopt(1);
    conf.bundle = args.get_nth_nonopt(2);

    for (util::ArgResult const* i = args.next_option();
         i != 0; i = args.next_option()) {
        switch (i->opt->sopt) {
        default:
            throw std::invalid_argument("Invalid option");
        }
    }

    return conf;
}

std::vector<std::string> tokenize(std::string const & str, char token) {
    std::vector<std::string> ret;

    std::size_t i = 0;
    for (std::size_t j = 0; j < str.size(); ++j) {
        if (str[j] != token) continue;

        ret.push_back(str.substr(i, j - i));
        i = j + 1;
    }
    ret.push_back(str.substr(i, str.size()));

    return ret;
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

    std::vector<mve::View::Ptr> const & views = scene->get_views();
    sfm::bundler::ViewportList viewports(views.size());

    std::unordered_map<std::string, int> name2vid;
    for (mve::View::Ptr const & view : views) {
        if (view == nullptr) continue;

        name2vid.emplace(view->get_name(), view->get_id());

        mve::View::ImageProxy const * proxy = view->get_image_proxy("original");
        int width = proxy->width;
        int height = proxy->height;

        sfm::bundler::Viewport & viewport = viewports[view->get_id()];
        viewport.features.width = width;
        viewport.features.height = height;

        mve::CameraInfo cam = view->get_camera();
        sfm::CameraPose * pose = &viewport.pose;

        fill_camera_pose(cam, width, height, pose);
    }

    std::ifstream in(args.gcpfile.c_str());
    if (!in.good()) {
        std::cerr << "Could not open gcp file: " << std::strerror(errno) << std::endl;
        std::exit(EXIT_FAILURE);
    }

    std::unordered_map<int, int> gcp2tid;
    sfm::bundler::TrackList tracks;

    int num_annos = 0;
    int num_found = 0;

    std::string line;
    while (in >> line) {
        num_annos += 1;

        std::vector<std::string> tokens = tokenize(line, ',');

        std::string name = tokenize(tokens[0],'.')[0];
        int id = std::stoi(tokens[1]);
        float x = std::stof(tokens[2]);
        float y = std::stof(tokens[3]);

        int tid = -1;
        {
            auto search = gcp2tid.find(id);
            if (search != gcp2tid.end()) {
                tid = search->second;
            }
        }

        int vid = -1;
        {
            auto search = name2vid.find(name);
            if (search != name2vid.end()) {
                vid = search->second;
            }
        }

        if (vid != -1) {
            num_found += 1;

            if (tid == -1) {
                tid = tracks.size();
                gcp2tid.insert({id, tid});
                tracks.emplace_back();
            }

            sfm::bundler::Viewport & viewport = viewports[vid];
            sfm::bundler::Track & track = tracks[tid];
            sfm::FeatureSet & features = viewport.features;
            viewport.track_ids.push_back(tid);

            float width = static_cast<float>(features.width);
            float height = static_cast<float>(features.height);
            float norm = std::max(width, height);

            x = (x + 0.5f - width / 2.0f) / norm;
            y = (y + 0.5f - height / 2.0f) / norm;

            int fid = features.positions.size();
            features.positions.emplace_back(x, y);
            features.colors.emplace_back(0, 255, 0);

            track.features.emplace_back(vid, fid);
        }
    }

    /* Initialize the incremental bundler and reconstruct first tracks. */
    sfm::bundler::Incremental::Options incremental_opts;
    incremental_opts.track_error_threshold_factor = 25.0f;
    incremental_opts.new_track_error_threshold = 0.01f;
    incremental_opts.min_triangulation_angle = MATH_DEG2RAD(1.0);
    incremental_opts.verbose_output = true;

    sfm::bundler::Incremental incremental(incremental_opts);
    incremental.initialize(&viewports, &tracks);
    incremental.triangulate_new_tracks(2);
    incremental.invalidate_large_error_tracks();

    mve::Bundle::Ptr bundle = incremental.create_bundle();
    mve::Bundle::Features const & features = bundle->get_features();

    for (auto & gcp_tid : gcp2tid) {
        std::cout << gcp_tid.first << ' '
            << math::Vec3f((float const *)features[gcp_tid.second].pos)
            << std::endl;
    }

    mve::save_mve_bundle(bundle, args.bundle);

    return EXIT_SUCCESS;
}
