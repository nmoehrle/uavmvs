#include <iostream>

#include "util/system.h"
#include "util/arguments.h"
#include "util/file_system.h"

#include "mve/scene.h"
#include "mve/bundle.h"
#include "mve/bundle_io.h"
#include "mve/mesh_io_ply.h"

struct Arguments {
    std::string cloud;
    std::string scene;
};

Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_minnum(2);
    args.set_nonopt_maxnum(2);
    args.set_usage("Usage: " + std::string(argv[0]) + " [OPTS] POINT_CLOUD SCENE");
    args.set_description("Create prebundle.sfm from point cloud and views in scene.");
    args.parse(argc, argv);

    Arguments conf;
    conf.cloud = args.get_nth_nonopt(0);
    conf.scene = args.get_nth_nonopt(1);

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

    mve::TriangleMesh::Ptr cloud;
    try {
        cloud = mve::geom::load_ply_mesh(args.cloud);
    } catch (std::exception& e) {
        std::cerr << "\tCould not load mesh: "<< e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }

    mve::Scene::Ptr scene;
    try {
        scene = mve::Scene::create(args.scene);
    } catch (std::exception& e) {
        std::cerr << "Could not open scene: " << e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }

    std::vector<mve::View::Ptr> const & views = scene->get_views();

    std::vector<math::Vec3f> vertices = cloud->get_vertices();

    mve::Bundle::Ptr bundle = mve::Bundle::create();
    mve::Bundle::Cameras & cameras = bundle->get_cameras();
    mve::Bundle::Features & features = bundle->get_features();

    features.resize(vertices.size());
    for (std::size_t i = 0; i < features.size(); ++i) {
        math::Vec3f const & vertex = vertices[i];
        math::Vec3f color = math::Vec3f(1.0f);
        std::copy(vertex.begin(), vertex.end(), features[i].pos);
        std::copy(color.begin(), color.end(), features[i].color);
    }

    cameras.resize(views.size());
    for (std::size_t i = 0; i < cameras.size(); ++i) {
        mve::View::Ptr const & view = views[i];
        if (view == nullptr) continue;

        cameras[i] = view->get_camera();
    }

    #pragma omp parallel for
    for (std::size_t i = 0; i < cameras.size(); ++i) {
        mve::CameraInfo const & cam = cameras[i];

        if (cam.flen == 0) continue;

        int width = 1920;
        int height = 1080;
        math::Matrix4f w2c;
        math::Matrix3f calib;
        math::Vec3f view_pos;

        cam.fill_calibration(calib.begin(), width, height);
        cam.fill_world_to_cam(w2c.begin());
        cam.fill_camera_pos(view_pos.begin());

        std::vector<mve::Bundle::Feature2D> features2d;

        for (std::size_t j = 0; j < vertices.size(); ++j) {
            math::Vec3f v = vertices[j];
            math::Vec3f v2c = view_pos - v;
            float n = v2c.norm();
            math::Vec3f pt = calib * w2c.mult(v, 1.0f);
            math::Vec2f p(pt[0] / pt[2] - 0.5f, pt[1] / pt[2] - 0.5f);
            if (p[0] < 0.0f || width <= p[0] || p[1] < 0.0f || height <= p[1]) continue;

            mve::Bundle::Feature2D feature2d;
            feature2d.view_id = i;
            feature2d.feature_id = j;

            features2d.push_back(feature2d);
        }

        //TODO check visibility

        #pragma omp critical
        for (std::size_t j = 0; j < features2d.size(); ++j) {
            mve::Bundle::Feature2D const & feature2d = features2d[j];
            if (feature2d.view_id == -1) continue;
            features[feature2d.feature_id].refs.push_back(feature2d);
        }
    }

    std::string filename = util::fs::join_path(args.scene, "synth_0.out");
    mve::save_mve_bundle(bundle, filename);

    return EXIT_SUCCESS;
}
