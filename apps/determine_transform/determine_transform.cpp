#include <iostream>

#include "util/system.h"
#include "util/arguments.h"
#include "util/choices.h"

#include "mve/scene.h"
#include "mve/bundle.h"
#include "mve/bundle_io.h"
#include "mve/mesh_io_ply.h"
#include "acc/bvh_tree.h"

struct Arguments {
    std::string bundle;
    std::string scene;
    std::string mesh;
};

Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_minnum(3);
    args.set_nonopt_maxnum(3);
    args.set_usage("Usage: " + std::string(argv[0]) + " [OPTS] BUNDLE SCENE MESH");
    args.set_description("Determine transform between bundle and scene.");
    args.parse(argc, argv);

    Arguments conf;
    conf.bundle = args.get_nth_nonopt(0);
    conf.scene = args.get_nth_nonopt(1);
    conf.mesh = args.get_nth_nonopt(2);

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

    mve::TriangleMesh::Ptr mesh;
    try {
        mesh = mve::geom::load_ply_mesh(args.mesh);
    } catch (std::exception& e) {
        std::cerr << "\tCould not load mesh: "<< e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }

    mve::Bundle::Ptr bundle;
    try {
        bundle = mve::load_mve_bundle(args.bundle);
    } catch (std::exception& e) {
        std::cerr << "\tCould not load bundle: "<< e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }

    std::vector<math::Vec3f> const & vertices = mesh->get_vertices();
    std::vector<uint> const & faces = mesh->get_faces();
    acc::BVHTree<uint, math::Vec3f> bvh_tree(faces, vertices);

    std::vector<mve::View::Ptr> const & views = scene->get_views();

    std::vector<mve::CameraInfo> cameras(views.size());
    for (std::size_t i = 0; i < cameras.size(); ++i) {
        mve::View::Ptr const & view = views[i];
        if (view == nullptr) continue;

        cameras[i] = view->get_camera();
    }

    int width = 1920;
    int height = 1080;

    mve::Bundle::Features & features = bundle->get_features();
    mve::Bundle::Cameras & bundle_cameras = bundle->get_cameras();

    #pragma omp parallel for
    for (std::size_t i = 0; i < features.size(); ++i) {
        math::Vec3f feature = math::Vec3f(features[i].pos);
        std::vector<mve::Bundle::Feature2D> & refs = features[i].refs;
        for (mve::Bundle::Feature2D & feature2d : refs) {
            mve::CameraInfo const & cam = bundle_cameras[feature2d.view_id];

            math::Vec3f cam_pos;
            math::Matrix4f w2c;
            math::Matrix3f calib;
            cam.fill_camera_pos(cam_pos.begin());
            cam.fill_calibration(calib.begin(), width, height);
            cam.fill_world_to_cam(w2c.begin());
            math::Vec3f pt = calib * w2c.mult(feature, 1.0f);
            math::Vec2f pos(pt[0] / pt[2] - 0.5f, pt[1] / pt[2] - 0.5f);

            std::copy(pos.begin(), pos.end(), feature2d.pos);
        }
    }

    #pragma omp parallel for
    for (std::size_t i = 0; i < features.size(); ++i) {
        math::Vec3f feature = math::Vec3f(features[i].pos);
        std::vector<math::Vec3f> projections;

        std::vector<mve::Bundle::Feature2D> const & refs = features[i].refs;
        for (mve::Bundle::Feature2D const & feature2d : refs) {
            mve::CameraInfo const & cam = cameras[feature2d.view_id];
            math::Vec2f pos(feature2d.pos);

            math::Vec3f cam_pos;
            math::Matrix3f invcalib;
            math::Matrix3f c2w_rot;
            cam.fill_camera_pos(cam_pos.begin());
            cam.fill_inverse_calibration(invcalib.begin(), width, height);
            cam.fill_cam_to_world_rot(c2w_rot.begin());

            math::Vec3f v = invcalib * math::Vec3f(pos[0], pos[1], 1.0f);

            acc::Ray<math::Vec3f> ray;
            ray.origin = cam_pos;
            ray.dir = c2w_rot.mult(v.normalized()).normalize();
            ray.tmin = 0.0f;
            ray.tmax = std::numeric_limits<float>::infinity();

            acc::BVHTree<uint, math::Vec3f>::Hit hit;
            if (!bvh_tree.intersect(ray, &hit)) continue;

            projections.push_back(cam_pos + hit.t * ray.dir);
        }

        if (projections.empty()) continue;

        for (int j = 0; j < 3; ++j) {
            std::stable_sort(projections.begin(), projections.end(),
                [j] (math::Vec3f const & r, math::Vec3f const & l) {
                    return r[j] > l[j];
                }
            );
        }
        math::Vec3f projection = projections[projections.size() / 2];

        #pragma omp critical
        std::cout << feature << " " << projection << std::endl;
    }

    return EXIT_SUCCESS;
}
