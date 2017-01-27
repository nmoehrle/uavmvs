#include <iostream>

#include "util/system.h"
#include "util/arguments.h"

#include "mve/image_io.h"
#include "mve/image_tools.h"

constexpr float inf = std::numeric_limits<float>::infinity();

struct Arguments {
    std::string in_image;
    std::string out_image;
    float min_value = -inf;
    float max_value = +inf;
    float clear_value = 0.0f;//NAN
};

Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_minnum(2);
    args.set_nonopt_maxnum(2);
    args.set_usage("Usage: " + std::string(argv[0]) + " [OPTS] IN_IMAGE OUT_IMAGE");
    args.set_description("Convert for mvei images...");
    args.add_option('\0', "max-value", true,
        "maximum value - values out of range will be set to clear-value");
    args.add_option('\0', "min-value", true,
        "minimum value - values out of range will be set to clear-value");
    args.parse(argc, argv);

    Arguments conf;
    conf.in_image = args.get_nth_nonopt(0);
    conf.out_image = args.get_nth_nonopt(1);

    for (util::ArgResult const* i = args.next_option();
         i != 0; i = args.next_option()) {
        switch (i->opt->sopt) {
        case '\0':
            if (i->opt->lopt == "max-value") {
                conf.max_value = i->get_arg<float>();
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

mve::ImageBase::Ptr
load_image(std::string const & filename) {
    std::string ext = util::string::lowercase(
        util::string::right(filename, 4));
    if (ext == ".pfm") {
        return mve::image::load_pfm_file(filename);
    } else if (ext == "mvei") {
        return mve::image::load_mvei_file(filename);
    } else {
        throw std::runtime_error("Not implemented");
    }
}

void
save_image(mve::ImageBase::Ptr base_image, std::string const & filename) {
    std::string ext = util::string::lowercase(
        util::string::right(filename, 4));
    if (ext == ".pfm") {
        mve::FloatImage::Ptr image;
        image = std::dynamic_pointer_cast<mve::FloatImage>(base_image);
        mve::image::save_pfm_file(image, filename);
    } else if (ext == "mvei") {
        mve::image::save_mvei_file(base_image, filename);
    } else if (ext == ".png") {
        mve::FloatImage::Ptr image;
        image = std::dynamic_pointer_cast<mve::FloatImage>(base_image);
        //mve::image::gamma_correct_srgb<float>(image);
        mve::ByteImage::Ptr byte_image;
        byte_image = mve::image::float_to_byte_image(image);
        mve::image::save_png_file(byte_image, filename);
    } else {
        throw std::runtime_error("Not implemented");
    }
}

int main(int argc, char **argv) {
    util::system::register_segfault_handler();
    util::system::print_build_timestamp(argv[0]);

    Arguments args = parse_args(argc, argv);

    mve::ImageBase::Ptr base_image = load_image(args.in_image);

    switch (base_image->get_type()) {
        case mve::IMAGE_TYPE_FLOAT:
        {
            mve::FloatImage::Ptr image;
            image = std::dynamic_pointer_cast<mve::FloatImage>(base_image);
            for (int i = 0; i < image->get_value_amount(); ++i) {
                float v = image->at(i);
                if (v < args.min_value || args.max_value < v) {
                    image->at(i) = args.clear_value;
                }
            }
        }
        break;
        default:
            throw std::runtime_error("Not implemented");
    }

    save_image(base_image, args.out_image);

    return EXIT_SUCCESS;
}
