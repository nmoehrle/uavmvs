#include <fstream>
#include <iostream>

#include "util/file_system.h"
#include "util/arguments.h"

#include "util/io.h"

#include "mve/scene.h"

#include "acc/bvh_tree.h"

struct Arguments {
    std::string scene;
    std::string out_file;
    std::string mesh;
};

Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_maxnum(2);
    args.set_nonopt_minnum(2);
    args.set_usage("Usage: " + std::string(argv[0]) + " [OPTS] IN_SCENE OUT_FILE");
    args.set_description("Writes out annotations");
    args.add_option('m', "mesh", true, "mesh for visibility checks");
    args.parse(argc, argv);

    Arguments conf;
    conf.scene = args.get_nth_nonopt(0);
    conf.out_file = args.get_nth_nonopt(1);

    for (util::ArgResult const* i = args.next_option();
         i != nullptr; i = args.next_option()) {
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
    Arguments args = parse_args(argc, argv);

    mve::Scene::Ptr scene;
    try {
        scene = mve::Scene::create(args.scene);
    } catch (std::exception& e) {
        std::cerr << "Could not open scene: " << e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }

    acc::BVHTree<uint, math::Vec3f>::Ptr bvh_tree;
    if (!args.mesh.empty()) {
        bvh_tree = load_mesh_as_bvh_tree(args.mesh);
    }

    std::vector<math::Vec3f> points = {
        {-9.02206, 120.179, 8.20717}, //hill
        {91.2755, 65.4411, 2.91817}, //in front of highrise
        {1.63729, -8.35646, 9.64557}, //center
        {-71.4888, -61.515, -0.428093}, //bottom
        {21.8781, -110.377, -4.49192}
    };

    std::vector<std::tuple<int, int, math::Vec2f> > observations;
    std::vector<mve::View::Ptr> const & views = scene->get_views();
    for (std::size_t i = 0; i < views.size(); ++i) {
        mve::View::Ptr const & view = views[i];
        if (view == nullptr) continue;

        mve::CameraInfo const& cam = view->get_camera();

        //mve::ByteImage::Ptr image = view->get_byte_image("undistorted");
        //
        //int width = image->width();
        //int height = image->height();
        int width = 3000;
        int height = 1500;
        float fnorm = std::max(width, height);

        math::Vec3f cam_pos;
        math::Vec3f view_dir;
        math::Matrix3f calib;
        math::Matrix4f w2c;
        cam.fill_camera_pos(cam_pos.begin());
        cam.fill_viewing_direction(view_dir.begin());
        cam.fill_calibration(calib.begin(), width, height);
        cam.fill_world_to_cam(w2c.begin());

        for (std::size_t j = 0; j < points.size(); ++j) {
            math::Vec3f const& point = points[j];
            math::Vec3f pt = calib * w2c.mult(point, 1.0f);
            math::Vec2f pos(pt[0] / pt[2], pt[1] / pt[2]);

            if (pos[0] < 10 || width - 10 <= pos[0]) continue;
            if (pos[1] < 10 || height - 10 <= pos[1]) continue;

            math::Vec3f dir = (point - cam_pos);
            float dist = dir.norm();
            dir /= dist;

            if (view_dir.dot(dir) < 0.0f)
                continue;

            acc::Ray<math::Vec3f> ray;
            ray.origin = cam_pos;
            ray.dir = dir;
            ray.tmin = 0.0f;
            ray.tmax = dist * 0.99f;

            if (bvh_tree->intersect(ray, nullptr)) continue;

            float fx = (pos[0] + 0.5f - width / 2.0f) / fnorm;
            float fy = (pos[1] + 0.5f - height / 2.0f) / fnorm;

            observations.emplace_back(j, i, math::Vec2f(fx, fy));
        }
#if 0
        mve::ByteImage::Ptr image = view->get_byte_image("annotation");
        int width = image->width();
        int height = image->height();
        float fnorm = std::max(width, height);

        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                math::Vec3uc col(&image->at(x, y, 0));
                float fx = (x + 0.5f - width / 2.0f) / fnorm;
                float fy = (y + 0.5f - height / 2.0f) / fnorm;

                int marker = -1;
                if (col == math::Vec3uc(255, 0, 255)) marker = 0;
                if (col == math::Vec3uc(255, 0, 0)) marker = 1;
                if (col == math::Vec3uc(0, 255, 0)) marker = 2;
                if (col == math::Vec3uc(0, 0, 255)) marker = 3;

                if (marker >= 0) {
                    std::cout << marker << ' ' << i << ' '
                        << fx << ' ' << fy << std::endl;
                }
            }
        }
#endif


    }


    std::ofstream out(args.out_file.c_str());
    if (!out.good()) {
        std::cerr << "Could not open file" << std::endl;
        std::exit(EXIT_FAILURE);
    }

    out << "MVE_SURVEY" << '\n'
        << points.size() << ' ' << observations.size() << '\n';
    for (std::size_t i = 0; i < points.size(); ++i) {
        out << points[i] << '\n';
    }
    for (std::size_t i = 0; i < observations.size(); ++i) {
        int point_id, view_id;
        math::Vec2f pos;
        std::tie(point_id, view_id, pos) = observations[i];
        out << point_id << ' ' << view_id << ' ' << pos << '\n';
    }
    out.close();

    return EXIT_SUCCESS;
}
