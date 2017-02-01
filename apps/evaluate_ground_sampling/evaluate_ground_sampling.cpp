#include <fstream>
#include <iostream>

#include "util/system.h"
#include "util/arguments.h"

#include "util/numpy_io.h"

#include "mve/scene.h"
#include "mve/image.h"

#include "mve/depthmap.h"

struct Arguments {
    std::string scene;
    std::string image;
    std::string file;
};

Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_minnum(3);
    args.set_nonopt_maxnum(3);
    args.set_usage("Usage: " + std::string(argv[0]) +
        " [OPTS] SCENE IMAGE FILE");
    args.set_description("Evaluates ground sampling of a mesh.");
    args.parse(argc, argv);

    Arguments conf;
    conf.scene = args.get_nth_nonopt(0);
    conf.image = args.get_nth_nonopt(1);
    conf.file = args.get_nth_nonopt(2);

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

    std::vector<float> gsds;

    std::vector<mve::View::Ptr> views = scene->get_views();
    #pragma omp parallel for
    for (std::size_t i = 0; i < views.size(); ++i) {
        mve::View::Ptr const & view = views[i];
        if (view == nullptr) continue;
        if (!view->has_image(args.image, mve::IMAGE_TYPE_FLOAT)) {
            std::cerr << "Warning view " << view->get_name()
                << " has no image " << args.image << std::endl;
            continue;
        }

        mve::FloatImage::Ptr dmap = view->get_float_image(args.image);

        mve::CameraInfo const & camera = view->get_camera();
        math::Vec3f origin;
        camera.fill_camera_pos(origin.begin());
        math::Matrix3f invcalib;
        camera.fill_inverse_calibration(invcalib.begin(),
            dmap->width(), dmap->height());

        std::vector<float> vgsds;
        for (int y = 1; y < dmap->height() - 1; ++y) {
            for (int x = 1; x < dmap->width() - 1; ++x) {
                float depth = dmap->at(x, y, 0);

                if (depth == 0.0) continue;

                vgsds.push_back(mve::geom::pixel_footprint(x, y, depth, invcalib));
            }
        }

        #pragma omp critical
        {
            gsds.insert(gsds.end(), vgsds.begin(), vgsds.end());
        }
    }

    save_numpy_file(gsds, args.file);

    return EXIT_SUCCESS;
}
