/*
 * Copyright (C) 2016-2018, Nils Moehrle
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include <iostream>

#include "util/system.h"
#include "util/arguments.h"

#include "mve/mesh_io_ply.h"

#include "geom/aabb.h"
#include "geom/aabb_io.h"


struct Arguments {
    std::string aabb;
    std::string omesh;
};

Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_minnum(2);
    args.set_nonopt_maxnum(2);
    args.set_usage("Usage: " + std::string(argv[0]) + " [OPTS] IN_AABB OUT_MESH");
    args.set_description("Generates a mesh for the given aabb");
    args.parse(argc, argv);

    Arguments conf;
    conf.aabb = args.get_nth_nonopt(0);
    conf.omesh = args.get_nth_nonopt(1);

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

    acc::AABB<math::Vec3f> aabb = load_aabb_from_file(args.aabb);
    mve::TriangleMesh::Ptr mesh = generate_aabb_mesh(aabb.min, aabb.max);

    mve::geom::SavePLYOptions opts;
    opts.write_vertex_normals = true;
    mve::geom::save_ply_mesh(mesh, args.omesh, opts);

    return EXIT_SUCCESS;
}
