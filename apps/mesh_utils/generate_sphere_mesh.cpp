#include <iostream>

#include "util/system.h"
#include "util/arguments.h"

#include "math/geometry.h"

#include "mve/mesh_io_ply.h"

#include "stat/statistics.h"

#include "geom/sphere.h"


struct Arguments {
    std::string omesh;
    float radius;
    uint subdivisions;
    bool show_info;
};

Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_minnum(1);
    args.set_nonopt_maxnum(1);
    args.set_usage("Usage: " + std::string(argv[0]) + " [OPTS] OUT_MESH");
    args.set_description("Generates a triangulated sphere by subdividing an icosahedron.");
    args.add_option('r', "radius", true, "radius of the sphere [1.0f]");
    args.add_option('s', "subdivisions", true, "number of subdivision iterations [2]");
    args.add_option('\0', "show-info", false, "show info");
    args.parse(argc, argv);

    Arguments conf;
    conf.omesh = args.get_nth_nonopt(0);
    conf.radius = 1.0f;
    conf.subdivisions = 2;
    conf.show_info = false;

    for (util::ArgResult const* i = args.next_option();
         i != 0; i = args.next_option()) {
        switch (i->opt->sopt) {
        case 'r':
            conf.radius = i->get_arg<float>();
        break;
        case 's':
            conf.subdivisions = i->get_arg<uint>();
        break;
        case '\0':
            if (i->opt->lopt == "show-info") {
                conf.show_info = true;
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

int main(int argc, char **argv) {
    util::system::register_segfault_handler();
    util::system::print_build_timestamp(argv[0]);

    Arguments args = parse_args(argc, argv);

    mve::TriangleMesh::Ptr mesh = generate_sphere_mesh(args.radius, args.subdivisions);

    if (args.show_info) {
        std::vector<uint> const & faces = mesh->get_faces();
        std::vector<math::Vec3f> const & verts = mesh->get_vertices();
        std::vector<float> areas(faces.size() / 3);

        for (std::size_t i = 0; i < areas.size(); ++i) {
            math::Vec3f v0 = verts[faces[i * 3 + 0]];
            math::Vec3f v1 = verts[faces[i * 3 + 1]];
            math::Vec3f v2 = verts[faces[i * 3 + 2]];
            areas[i] = std::abs(math::geom::triangle_area(v0, v1, v2));
        }

        float mean, var;
        std::tie(mean, var) = stat::moments(areas);
        float cv = std::sqrt(var) / mean;

        std::cout << "Mean: " << mean << " Var: " << var
            << " CV: " << cv << std::endl;
    }

    mve::geom::SavePLYOptions opts;
    opts.write_vertex_normals = true;
    mve::geom::save_ply_mesh(mesh, args.omesh, opts);

    return EXIT_SUCCESS;
}
