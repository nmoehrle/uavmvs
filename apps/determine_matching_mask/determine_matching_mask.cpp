#include <iostream>

#include "util/system.h"
#include "util/arguments.h"

#include "util/io.h"

#include "mve/scene.h"
#include "mve/image_io.h"
#include "mve/mesh_io_ply.h"

#include "acc/bvh_tree.h"

constexpr float inf = std::numeric_limits<float>::infinity();

struct Arguments {
    std::string cloud;
    std::string mesh;
    std::string scene;
    std::string matrix;
};

Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_minnum(3);
    args.set_nonopt_maxnum(3);
    args.set_usage("Usage: " + std::string(argv[0]) + " [OPTS] SCENE CLOUD OUT_MATRIX");
    args.set_description("Template app");
    args.add_option('m', "mesh", true, "mesh for visibility checks");
    args.parse(argc, argv);

    Arguments conf;
    conf.scene= args.get_nth_nonopt(0);
    conf.cloud= args.get_nth_nonopt(1);
    conf.matrix = args.get_nth_nonopt(2);

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

int main(int argc, char **argv) {
    util::system::register_segfault_handler();
    util::system::print_build_timestamp(argv[0]);

    Arguments args = parse_args(argc, argv);

    mve::TriangleMesh::Ptr cloud;
    try {
        cloud = mve::geom::load_ply_mesh(args.cloud);
    } catch (std::exception& e) {
        std::cerr << "\tCould not load cloud: "<< e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }
    std::vector<math::Vec3f> const & verts = cloud->get_vertices();

    mve::Scene::Ptr scene;
    try {
        scene = mve::Scene::create(args.scene);
    } catch (std::exception& e) {
        std::cerr << "Could not open scene: " << e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }

    std::vector<mve::CameraInfo> cams;
    for (mve::View::Ptr view : scene->get_views()) {
        if (view == nullptr) continue;
        if (!view->is_camera_valid()) continue;
        cams.push_back(view->get_camera());
    }

    std::size_t const num_cams = cams.size();

    if (num_cams == 0) {
        std::cerr << "Scene does not contain valid cameras." << std::endl;
        std::exit(EXIT_FAILURE);
    }

    int width = 1920;
    int height = 1080;

    acc::BVHTree<uint, math::Vec3f>::Ptr bvh_tree;
    if (!args.mesh.empty()) {
        bvh_tree = load_mesh_as_bvh_tree(args.mesh);
    }

    std::vector<std::vector<uint> > vis(num_cams);

    #pragma omp parallel for
    for (std::size_t i = 0; i < num_cams; ++i) {
        mve::CameraInfo const & cam = cams[i];
        math::Matrix3f calib;
        cam.fill_calibration(calib.begin(), width, height);
        math::Matrix4f w2c;
        cam.fill_world_to_cam(w2c.begin());
        math::Vec3f view_pos;
        cam.fill_camera_pos(view_pos.begin());

        std::vector<uint> & vvis = vis[i];

        for (std::size_t j = 0; j < verts.size(); ++j) {
            math::Vec3f v = verts[j];

            math::Vec3f v2c = view_pos - v;
            float n = v2c.norm();
            math::Vec3f pt = calib * w2c.mult(v, 1.0f);
            math::Vec2f p(pt[0] / pt[2] - 0.5f, pt[1] / pt[2] - 0.5f);

            if (p[0] < 0.0f || width <= p[0] || p[1] < 0.0f || height <= p[1]) continue;

            if (bvh_tree != nullptr) {
                acc::Ray<math::Vec3f> ray;
                ray.origin = v + v2c * 0.01f;
                ray.dir = v2c / n;
                ray.tmin = n * 0.01f;
                ray.tmax = n;

                if (bvh_tree->intersect(ray)) continue;
            }

            vvis.push_back(j);
        }
    }

    mve::FloatImage::Ptr mat = mve::FloatImage::create(num_cams, num_cams, 1);

    #pragma omp parallel for schedule(dynamic)
    for (std::size_t i = 0; i < num_cams; ++i) {
        for (std::size_t j = i + 1; j < num_cams; ++j) {
            auto iit = vis[i].begin();
            auto iend = vis[i].end();
            auto jit = vis[j].begin();
            auto jend = vis[j].end();

            uint cnt = 0;
            while (iit != iend && jit != jend) {
                int vi = *iit;
                int vj = *jit;
                if (vi == vj) {
                    iit += 1;
                    jit += 1;
                    cnt += 1;
                } else {
                    if (vi < vj) {
                        iit += 1;
                    } else {
                        jit += 1;
                    }
                }
            }

            mat->at(i, j, 0) = cnt;
            mat->at(j, i, 0) = cnt;
        }
    }

    mve::image::save_pfm_file(mat, args.matrix);

    return EXIT_SUCCESS;
}
