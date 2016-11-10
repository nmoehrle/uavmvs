#include <iostream>

#include "util/system.h"
#include "util/arguments.h"

#include "mve/mesh_io.h"

#include "geom/cloud_io.h"

struct Arguments {
    std::string in_cloud;
    std::string out_cloud;
};

Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_minnum(2);
    args.set_nonopt_maxnum(2);
    args.set_usage("Usage: " + std::string(argv[0]) + " [OPTS] IN_CLOUD OUT_CLOUD");
    args.set_description("Converts point cloud formats");
    args.parse(argc, argv);

    Arguments conf;
    conf.in_cloud = args.get_nth_nonopt(0);
    conf.out_cloud = args.get_nth_nonopt(1);

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

    std::string ext = util::string::right(args.in_cloud, 3);

    mve::TriangleMesh::Ptr mesh;
    try {
        if (ext == "ptx") {
            mesh = load_ptx_cloud(args.in_cloud);
        } else {
            mesh = mve::geom::load_mesh(args.in_cloud);
        }
    } catch (std::exception& e) {
        std::cerr << "Could not load mesh: " << e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }

    mve::geom::save_mesh(mesh, args.out_cloud);

    return EXIT_SUCCESS;
}
