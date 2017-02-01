#include <iostream>

#include "util/arguments.h"
#include "mve/image_io.h"

#include "col/mpl_viridis.h"
#include "col/dcm_coolwarm.h"
#include "util/choices.h"

enum ColorMap {
    VIRIDIS = 0,
    COOLWARM = 1,
    RED = 2
};

template <> inline
const std::vector<std::string> choice_strings<ColorMap>() {
    return {"viridis", "coolwarm"};
}

math::Vec3f parse_color(std::string const & str) {
    unsigned num;
    std::stringstream ss(str);
    ss >> std::hex >> num;
    float r = (num >>  0 & 0xFF) / 255.0f;
    float g = (num >>  8 & 0xFF) / 255.0f;
    float b = (num >> 16 & 0xFF) / 255.0f;
    return math::Vec3f(r, g, b);
}

struct Arguments {
    std::string in_image;
    std::string out_image;
    ColorMap colormap;
    math::Vec3f ccolor;
};

Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_maxnum(2);
    args.set_nonopt_minnum(2);
    args.set_usage("Usage: " + std::string(argv[0]) + " [OPTS] IN_IMAGE OUT_IMAGE");
    args.set_description("Values out of range [0:1] will be mapped to ccolor");
    args.add_option('c', "ccolor", true, "colorvalue for out of range values [FF00FF]");
    args.add_option('m', "colormap", true, "colorize according to given colormap "
        + choices<ColorMap>(VIRIDIS));
    args.parse(argc, argv);

    Arguments conf;
    conf.in_image= args.get_nth_nonopt(0);
    conf.out_image = args.get_nth_nonopt(1);
    conf.colormap = VIRIDIS;
    conf.ccolor = math::Vec3f(1.0f, 0.0f, 1.0f);

    for (util::ArgResult const* i = args.next_option();
         i != 0; i = args.next_option()) {
        switch (i->opt->sopt) {
        case 'c':
            conf.ccolor = parse_color(i->arg);
        break;
        case 'm':
            conf.colormap = parse_choice<ColorMap>(i->arg);
        break;
        default:
            throw std::invalid_argument("Invalid option");
        }
    }

    return conf;
}

int main(int argc, char **argv) {
    Arguments args = parse_args(argc, argv);

    mve::FloatImage::Ptr in_image;
    try {
        in_image = mve::image::load_pfm_file(args.in_image);
    } catch (std::exception& e) {
        std::cerr << "Could not load image: " << e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }

    if (in_image->channels() != 1) {
        std::cerr << "Image has more than one channel."<< std::endl;
        std::exit(EXIT_FAILURE);
    }

    float (*colormap)[3];
    switch(args.colormap) {
        case VIRIDIS: colormap = col::maps::srgb::viridis; break;
        case COOLWARM: colormap = col::maps::coolwarm; break;
        default: colormap = col::maps::srgb::viridis;
    }

    mve::FloatImage::Ptr out_image;
    out_image = mve::FloatImage::create(in_image->width(), in_image->height(), 3);

    for (int i = 0; i < in_image->get_value_amount(); i++){
        float value = in_image->at(i);

        math::Vec3f color(args.ccolor);
        if (0.0f <= value && value <= 1.0f) {
            std::uint8_t lidx = std::floor(value * 255.0f);
            float t = value * 255.0f - lidx;
            std::uint8_t hidx = lidx == 255 ? 255 : lidx + 1;

            math::Vec3f lc(colormap[lidx]);
            math::Vec3f hc(colormap[hidx]);
            for (int j = 0; j < 3; ++j) {
                out_image->at(i, j) = (1.0f - t) * lc[j] + t * hc[j];
            }
        } else {
            std::copy(color.begin(), color.end(), &out_image->at(i, 0));
        }
    }

    mve::image::save_pfm_file(out_image, args.out_image);
}
