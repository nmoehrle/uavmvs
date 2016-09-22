#include <iostream>

#include "fmt/format.h"

#include "util/system.h"
#include "util/arguments.h"
#include "util/choices.h"

#include "mve/image_io.h"
#include "mve/mesh_io_ply.h"
#include "mve/marching_cubes.h"

#include "geom/volume_io.h"
#include "geom/grid_mesh_mc_accessor.h"

enum Format {
    IMAGES = 0,
    SURFACE = 1,
    CLOUD = 2
};

template <> inline
const std::vector<std::string> choice_strings<Format>() {
    return {"images", "surface", "cloud"};
}

struct Arguments {
    std::string in_volume;
    std::string out_prefix;
    Format format;
};

Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_minnum(2);
    args.set_nonopt_maxnum(2);
    args.set_usage("Usage: " + std::string(argv[0]) + " [OPTS] IN_VOLUME OUT_PREFIX");
    args.set_description("Conversion app for volumes a la image magicks convert");
    args.add_option('f', "format", true, "output format "
        + choices<Format>(IMAGES));
    args.parse(argc, argv);

    Arguments conf;
    conf.in_volume = args.get_nth_nonopt(0);
    conf.out_prefix = args.get_nth_nonopt(1);
    conf.format = IMAGES;

    for (util::ArgResult const* i = args.next_option();
         i != 0; i = args.next_option()) {
        switch (i->opt->sopt) {
        case 'f':
            conf.format = parse_choice<Format>(i->arg);
        break;
        default:
            throw std::invalid_argument("Invalid option");
        }
    }

    return conf;
}

//TODOs
//expose iso
//determine format on output schema

int main(int argc, char **argv) {
    util::system::register_segfault_handler();
    util::system::print_build_timestamp(argv[0]);

    Arguments args = parse_args(argc, argv);

    Volume<std::uint32_t>::Ptr volume;
    try {
        volume = load_volume<std::uint32_t>(args.in_volume);
    } catch (std::exception& e) {
        std::cerr << "Could not load volume: " << e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }

    std::uint32_t width = volume->width();
    std::uint32_t height = volume->height();
    std::uint32_t depth = volume->depth();

    if (args.format == CLOUD || args.format == SURFACE) {
        mve::TriangleMesh::Ptr ocloud = mve::TriangleMesh::create();
        std::vector<math::Vec3f> overts = ocloud->get_vertices();
        std::vector<float> ovalues = ocloud->get_vertex_values();
        overts.reserve(volume->num_positions());
        ovalues.reserve(volume->num_positions());

        for (std::uint32_t z = 0; z < depth; ++z) {
            #pragma omp parallel for ordered
            for (std::uint32_t y = 0; y < height; ++y) {
                for (std::uint32_t x = 0; x < width; ++x) {
                    float value = -1.0f;

                    mve::FloatImage::Ptr image = volume->at(x, y, z);
                    if (image != nullptr) {
                        std::vector<float> const & values = image->get_data();
                        value = *std::max_element(values.begin(), values.end());
                    }

                    #pragma omp ordered
                    {
                        overts.push_back(volume->position(x, y, z));
                        ovalues.push_back(value);
                    }
                }
            }
        }

        if (args.format == CLOUD) {
            mve::geom::SavePLYOptions options;
            options.write_vertex_values = true;

            mve::geom::save_ply_mesh(ocloud, args.out_prefix + ".ply", options);
        }

        if (args.format == SURFACE) {
            float iso = 0.55f;
            std::for_each(ovalues.begin(), ovalues.end(), [iso] (float & v) { v -= iso; });
            GridMeshMCAccessor accessor(ocloud, width, height, depth);
            mve::TriangleMesh::Ptr omesh = mve::geom::marching_cubes(accessor);
            mve::geom::save_ply_mesh(omesh, args.out_prefix + ".ply");
        }
    }

    if (args.format == IMAGES) {
        for (std::uint32_t z = 0; z < depth; ++z) {
            mve::FloatImage::Ptr oimage = mve::FloatImage::create(width, height, 1);
            #pragma omp parallel for
            for (std::uint32_t y = 0; y < height; ++y) {
                for (std::uint32_t x = 0; x < width; ++x) {
                    mve::FloatImage::Ptr image = volume->at(x, y, z);
                    if (image != nullptr) {
                        std::vector<float> const & values = image->get_data();
                        oimage->at(x, y, 0) = *std::max_element(values.begin(), values.end());
                    } else {
                        oimage->at(x, y, 0) = -1.0f;
                    }
                }
            }
            std::string filename = fmt::format("{}-{:04d}.pfm", args.out_prefix, z);
            mve::image::save_pfm_file(oimage, filename);
        }
    }

    return EXIT_SUCCESS;
}
