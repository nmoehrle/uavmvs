#include <iostream>

#include "util/system.h"
#include "util/arguments.h"

#include "geom/volume_io.h"

#include "utp/trajectory_io.h"

struct Arguments {
    std::string guidance_volume;
    std::string out_trajectory;
    uint num_views;
    float focal_length;
};

Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_minnum(2);
    args.set_nonopt_maxnum(2);
    args.set_usage("Usage: " + std::string(argv[0])
        + " [OPTS] GUIDANCE_VOLUME OUT_TRAJECTORY");
    args.set_description("Samples the guidance volume to create an initial trajectory");
    args.add_option('\0', "focal-length", true, "camera focal length [0.86]");
    args.add_option('n', "num-views", true, "number of views [500]");
    args.parse(argc, argv);

    Arguments conf;
    conf.guidance_volume = args.get_nth_nonopt(0);
    conf.out_trajectory = args.get_nth_nonopt(1);
    conf.num_views = 500;
    conf.focal_length = 0.86f;

    for (util::ArgResult const* i = args.next_option();
         i != 0; i = args.next_option()) {
        switch (i->opt->sopt) {
        case 'n':
            conf.num_views = i->get_arg<uint>();
        break;
        case '\0':
            if (i->opt->lopt == "focal-length") {
                conf.focal_length = i->get_arg<float>();
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

float const pi = std::acos(-1.0f);

int main(int argc, char **argv) {
    util::system::register_segfault_handler();
    util::system::print_build_timestamp(argv[0]);

    Arguments args = parse_args(argc, argv);

    Volume<std::uint32_t>::Ptr volume;
    try {
        volume = load_volume<std::uint32_t>(args.guidance_volume);
    } catch (std::exception& e) {
        std::cerr << "Could not load volume: " << e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }

    mve::CameraInfo cam;
    cam.flen = args.focal_length;

    std::vector<mve::CameraInfo> trajectory;

    std::mt19937 gen(12345);
    std::uniform_int_distribution<std::uint32_t> dist(0, volume->num_positions() - 1);
    while (trajectory.size() < args.num_views) {
        std::uint32_t idx = dist(gen);
        mve::FloatImage::Ptr image = volume->at(idx);
        if (image == nullptr) continue;

        std::uniform_real_distribution<float> rdis(0.0f, 1.0f);
        std::uniform_int_distribution<int> idis(0, image->get_value_amount() - 1);

        int i = idis(gen);
        if(image->at(i) < rdis(gen)) continue;

        int x = i % image->width();
        int y = i / image->width();

        float theta = (0.5f + (y / (float) image->height()) / 2.0f) * pi;
        float phi = (x / (float) image->width()) * 2.0f * pi;

        math::Matrix3f rot = utp::rotation_from_spherical(theta, phi);

        math::Vec3f offset = math::Vec3f(rdis(gen), rdis(gen), rdis(gen)) / 100.0f;
        math::Vec3f pos = volume->position(idx) + offset;
        math::Vec3f trans = -rot * pos;

        std::copy(trans.begin(), trans.end(), cam.trans);
        std::copy(rot.begin(), rot.end(), cam.rot);

        trajectory.push_back(cam);
    }

    utp::save_trajectory(trajectory, args.out_trajectory);

    return EXIT_SUCCESS;
}
