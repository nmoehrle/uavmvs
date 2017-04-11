#include <iostream>

#include "util/system.h"
#include "util/arguments.h"

#include "util/matrix_io.h"

#include "mve/mesh_io_ply.h"

#include "geom/icp.h"

struct Arguments {
    std::string mesh;
    std::string rmesh;
    std::string transform;
};

Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_minnum(2);
    args.set_nonopt_maxnum(2);
    args.set_usage("Usage: " + std::string(argv[0]) + " [OPTS] MESH REFERENCE_MESH");
    args.set_description("Estimates transform of a mesh w.r.t a reference mesh"
        "trough icp (meshes should be roughly aligned).");
    args.add_option('t', "transform", true, "save transform to matrix file");
    args.parse(argc, argv);

    Arguments conf;
    conf.mesh = args.get_nth_nonopt(0);
    conf.rmesh = args.get_nth_nonopt(1);

    for (util::ArgResult const* i = args.next_option();
         i != 0; i = args.next_option()) {
        switch (i->opt->sopt) {
        case 't':
            conf.transform = i->arg;
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

    mve::TriangleMesh::Ptr mesh, rmesh;
    try {
        mesh = mve::geom::load_ply_mesh(args.mesh);
        rmesh = mve::geom::load_ply_mesh(args.rmesh);
    } catch (std::exception& e) {
        std::cerr << "\tCould not load mesh: "<< e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }

    float avg_dist = 1.0f;
    math::Matrix4f T = estimate_transform(mesh, rmesh, 100, &avg_dist);

    std::cout << "Average distance between surfaces " << avg_dist << std::endl;

    if (!args.transform.empty()) {
        save_matrix_to_file(T, args.transform);
    } else {
        std::cout << T << std::endl;
    }

    return EXIT_SUCCESS;
}
