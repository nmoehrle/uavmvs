#include <iostream>

#include "fmt/format.h"

#include "util/system.h"
#include "util/arguments.h"
#include "util/choices.h"

#include "mve/image_io.h"
#include "mve/mesh_io_ply.h"
#include "mve/marching_cubes.h"

#include "grid_mesh_mc_accessor.h"

enum Format {
    IMAGES = 0,
    SURFACE = 1
};

template <> inline
const std::vector<std::string> choice_strings<Format>() {
    return {"images", "surface"};
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

    mve::TriangleMesh::Ptr volume;
    try {
        volume = mve::geom::load_ply_mesh(args.in_volume);
    } catch (std::exception& e) {
        std::cerr << "\tCould not load mesh: "<< e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }

    std::vector<uint> const & faces = volume->get_faces();
    std::vector<math::Vec3f> const & verts = volume->get_vertices();
    std::vector<float> & values = volume->get_vertex_values();

    if (faces.size() != 3) {
        std::cerr << "\tInvalid volume - dimensions missing" << std::endl;
        std::exit(EXIT_FAILURE);
    }
    uint width = faces[0];
    uint height = faces[1];
    uint depth = faces[2];
    if (width * height * depth != (verts.size() & values.size())) {
        std::cerr << "\tInvalid volume - dimensions wrong" << std::endl;
        std::exit(EXIT_FAILURE);
    }

    if (args.format == IMAGES) {
        for (uint i = 0; i < depth; ++i) {
            mve::FloatImage::Ptr image = mve::FloatImage::create(width, height, 1);
            for (uint y = 0; y < height; ++y) {
                for (uint x = 0; x < width; ++x) {
                    image->at(x, y, 0) = values[(i * height + y) * width + x];
                }
            }
            std::string filename = fmt::format("{}-{:04d}.pfm", args.out_prefix, i);
            mve::image::save_pfm_file(image, filename);
        }
    }

    if (args.format == SURFACE) {
        float iso = 0.75f;
        std::for_each(values.begin(), values.end(), [iso] (float & v) { v -= iso; });
        GridMeshMCAccessor accessor(volume, width, height, depth);
        mve::TriangleMesh::Ptr mesh = mve::geom::marching_cubes(accessor);
        mve::geom::save_ply_mesh(mesh, args.out_prefix + ".ply");
    }

    return EXIT_SUCCESS;
}
