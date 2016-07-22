#include <iostream>

#include "util/system.h"
#include "util/arguments.h"
#include "util/file_system.h"

#include "mve/scene.h"
#include "mve/bundle.h"
#include "mve/bundle_io.h"
#include "mve/mesh_io_ply.h"

#include "acc/bvh_tree.h"

struct Arguments {
    std::string mesh;
    std::string cloud;
    std::string scene;
};

Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_minnum(2);
    args.set_nonopt_maxnum(2);
    args.set_usage("Usage: " + std::string(argv[0]) + " [OPTS] POINT_CLOUD SCENE");
    args.set_description("Create synth_0.out from point cloud and views in scene.");
    args.add_option('m', "mesh", true, "mesh for visibility checks");
    args.parse(argc, argv);

    Arguments conf;
    conf.cloud = args.get_nth_nonopt(0);
    conf.scene = args.get_nth_nonopt(1);

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

acc::BVHTree<uint, math::Vec3f>::Ptr
load_mesh_as_bvh_tree(std::string const & path)
{
    mve::TriangleMesh::Ptr mesh;
    try {
        mesh = mve::geom::load_ply_mesh(path);
    } catch (std::exception& e) {
        std::cerr << "\tCould not load mesh: "<< e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }

    std::vector<math::Vec3f> const & vertices = mesh->get_vertices();
    std::vector<uint> const & faces = mesh->get_faces();
    return acc::BVHTree<uint, math::Vec3f>::create(faces, vertices);
}

int main(int argc, char **argv) {
    util::system::register_segfault_handler();
    util::system::print_build_timestamp(argv[0]);

    Arguments args = parse_args(argc, argv);

    acc::BVHTree<uint, math::Vec3f>::Ptr bvh_tree;
    if (!args.mesh.empty()) {
        bvh_tree = load_mesh_as_bvh_tree(args.mesh);
    }

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

    std::vector<math::Vec3f> const & vertices = cloud->get_vertices();
    std::vector<math::Vec3f> const & normals = cloud->get_vertex_normals();

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
            math::Vec3f n = normals[j];
            math::Vec3f v2c = view_pos - v;
            float l = v2c.norm();
            math::Vec3f v2cn = v2c / l;

            math::Vec3f pt = calib * w2c.mult(v, 1.0f);
            math::Vec2f p(pt[0] / pt[2] - 0.5f, pt[1] / pt[2] - 0.5f);
            if (p[0] < 0.0f || width <= p[0] || p[1] < 0.0f || height <= p[1]) continue;

            if (bvh_tree != nullptr) {
                acc::Ray<math::Vec3f> ray;
                ray.origin = v;
                ray.dir = v2cn;
                ray.tmin = 0.001f * l;
                ray.tmax = l;

                if (bvh_tree->intersect(ray)) continue;
            }

            mve::Bundle::Feature2D feature2d;
            feature2d.view_id = i;
            feature2d.feature_id = j;

            features2d.push_back(feature2d);
        }

        #pragma omp critical
        for (std::size_t j = 0; j < features2d.size(); ++j) {
            mve::Bundle::Feature2D const & feature2d = features2d[j];
            features[feature2d.feature_id].refs.push_back(feature2d);
        }
    }

    mve::Bundle::Features new_features;
    new_features.reserve(features.size());

    for (std::size_t i = 0; i < features.size(); ++i) {
        mve::Bundle::Feature3D const & f = features[i];
        if (f.refs.size() < 2) continue;
        new_features.push_back(f);
    }

    features.swap(new_features);

    scene->set_bundle(bundle);
    scene->save_bundle();

    return EXIT_SUCCESS;
}
