#include <fstream>
#include <iostream>
#include <unordered_set>

#include "util/file_system.h"
#include "util/arguments.h"

#include "math/transform.h"

#include "mve/scene.h"

#include "acc/kd_tree.h"

struct Arguments {
    std::string in_scene;
    std::string gt_scene;
};

Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_maxnum(2);
    args.set_nonopt_minnum(2);
    args.set_usage("Usage: " + std::string(argv[0]) + " [OPTS] IN_SCENE GT_SCENE");
    args.set_description("TODO");
    args.parse(argc, argv);

    Arguments conf;
    conf.in_scene = args.get_nth_nonopt(0);
    conf.gt_scene = args.get_nth_nonopt(1);

    for (util::ArgResult const* i = args.next_option();
         i != nullptr; i = args.next_option()) {
        switch (i->opt->sopt) {
        default:
            throw std::invalid_argument("Invalid option");
        }
    }

    return conf;
}

void
fill(std::vector<mve::View::Ptr> const & views,
    std::vector<math::Vec3f> * cam_poss, std::vector<math::Vec3f> * view_dirs)
{
    cam_poss->resize(views.size());
    view_dirs->resize(views.size());

    std::size_t num_valid = 0;
    for (std::size_t i = 0; i < views.size(); ++i) {
        mve::View::Ptr const & view = views[i];
        if (view == nullptr) continue;

        mve::CameraInfo const& cam = view->get_camera();
        if (cam.flen == 0.0f) continue;

        num_valid += 1;
        cam.fill_camera_pos(cam_poss->at(i).begin());
        cam.fill_viewing_direction(view_dirs->at(i).begin());
    }

    std::cout << num_valid << ' ' << views.size() << std::endl;
}

int main(int argc, char **argv) {
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

    std::vector<math::Vec3f> in_cam_poss;
    std::vector<math::Vec3f> gt_cam_poss;
    std::vector<math::Vec3f> in_view_dirs;
    std::vector<math::Vec3f> gt_view_dirs;

    fill(in_scene->get_views(), &in_cam_poss, &in_view_dirs);
    fill(gt_scene->get_views(), &gt_cam_poss, &gt_view_dirs);


    std::vector<math::Vec3f> v0, v1;

    acc::KDTree<3, uint> kd_tree(in_cam_poss);
    for (std::size_t i = 0; i < gt_cam_poss.size(); ++i) {
        math::Vec3f gt_cam_pos = gt_cam_poss[i];
        //math::Vec3f gt_view_dir = gt_view_dirs[i];
        math::Vec3f gt_p = gt_cam_pos + gt_view_dirs[i] * 50.0f;

        std::vector<std::pair<uint, float> > nns;
        kd_tree.find_nns(gt_cam_pos, 5, &nns);
        float lowest = 1.0f/0.0f;
        uint best = -1;
        for (std::size_t j = 0; j < nns.size(); ++j) {
            uint id = nns[j].first;
            //float dist = nns[j].second;
            math::Vec3f in_cam_pos = in_cam_poss[id];
            math::Vec3f in_p = in_cam_pos + in_view_dirs[id] * 50.0f;
            //float angle = in_view_dirs[id].dot(gt_view_dir);
            //float error = dist * (2.0 - angle);
            float error = (gt_cam_pos - in_cam_pos).norm() + (gt_p - in_p).norm();
            if (error < lowest) {
                lowest = error;
                best = id;
            }
        }

        v0.push_back(gt_cam_pos);
        v1.push_back(in_cam_poss[best]);
    }

    math::Matrix3f R;
    math::Vec3f t;
    float s;

    std::cout << "Refinement transform: " << std::endl;
    math::determine_transform(v0, v1, &R, &s, &t);
    std::cout << R << s << '\n' << t << std::endl;
    std::unordered_set<uint> set;

    for (std::size_t i = 0; i < gt_cam_poss.size(); ++i) {
        //math::Vec3f gt_cam_pos = R * s * gt_cam_poss[i] + t;
        math::Vec3f gt_cam_pos = gt_cam_poss[i];
        //math::Vec3f gt_view_dir = gt_view_dirs[i];
        //math::Vec3f gt_p = gt_cam_pos + (R * gt_view_dirs[i]) * 50.0f;
        math::Vec3f gt_p = gt_cam_pos + gt_view_dirs[i] * 50.0f;

        std::vector<std::pair<uint, float> > nns;
        kd_tree.find_nns(gt_cam_pos, 5, &nns);
        float lowest = 1.0f/0.0f;
        uint best = -1;
        for (std::size_t j = 0; j < nns.size(); ++j) {
            uint id = nns[j].first;
            //float dist = nns[j].second;
            math::Vec3f in_cam_pos = in_cam_poss[id];
            math::Vec3f in_p = in_cam_pos + in_view_dirs[id] * 50.0f;
            //float angle = in_view_dirs[id].dot(gt_view_dir);
            //float error = dist * (2.0 - angle);
            float error = (gt_cam_pos - in_cam_pos).norm() + (gt_p - in_p).norm();
            if (error < lowest) {
                lowest = error;
                best = id;
            }
        }


        if (!set.insert(best).second) {
            std::cout << "WARNING duplicate" << std::endl;
        }

        std::cout << i << ' ' << best << ' ' << lowest << ' '
            << in_scene->get_view_by_id(best)->get_name() << std::endl;
    }

    return EXIT_SUCCESS;
}
