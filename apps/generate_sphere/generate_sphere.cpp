#include <iostream>

#include "util/system.h"
#include "util/arguments.h"

#include "mve/mesh_io_ply.h"

struct Arguments {
    std::string omesh;
    float radius;
    uint subdivisions;
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
    args.parse(argc, argv);

    Arguments conf;
    conf.omesh = args.get_nth_nonopt(0);
    conf.radius = 1.0f;
    conf.subdivisions = 2;

    for (util::ArgResult const* i = args.next_option();
         i != 0; i = args.next_option()) {
        switch (i->opt->sopt) {
        case 'r':
            conf.radius = i->get_arg<float>();
        break;
        case 's':
            conf.subdivisions = i->get_arg<uint>();
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

    /* Derived from mve/apps/umve/scene_addins/addin_sphere_creator.cc */

    /* Initialize icosahedron */
    std::vector<math::Vec3f> verts = {
        {0.0f, -0.5257311f, 0.8506508f},
        {0.0f, 0.5257311f, 0.8506508f},
        {0.0f, -0.5257311f, -0.8506508f},
        {0.0f, 0.5257311f, -0.8506508f},
        {0.8506508f, 0.0f, 0.5257311f},
        {0.8506508f, 0.0f, -0.5257311f},
        {-0.8506508f, 0.0f, 0.5257311f},
        {-0.8506508f, 0.0f, -0.5257311f},
        {0.5257311f, 0.8506508f, 0.0f},
        {0.5257311f, -0.8506508f, 0.0f},
        {-0.5257311f, 0.8506508f, 0.0f},
        {-0.5257311f, -0.8506508f, 0.0f}
    };

    std::vector<uint> faces = {
        0, 4, 1,
        0, 9, 4,
        9, 5, 4,
        4, 5, 8,
        4, 8, 1,
        8, 10, 1,
        8, 3, 10,
        5, 3, 8,
        5, 2, 3,
        2, 7, 3,
        7, 10, 3,
        7, 6, 10,
        7, 11, 6,
        11, 0, 6,
        0, 1, 6,
        6, 1, 10,
        9, 0, 11,
        9, 11, 2,
        9, 2, 5,
        7, 2, 11,
    };

    /* Icosahedron subdivision. */
    for (int i = 0; i < args.subdivisions; ++i) {
        /* Walk over faces and generate vertices. */
        typedef std::pair<uint, uint> Edge;
        typedef std::pair<Edge, uint> MapValueType;
        std::map<Edge, uint> edge_map;
        for (std::size_t j = 0; j < faces.size(); j += 3)
        {
            uint v0 = faces[j + 0];
            uint v1 = faces[j + 1];
            uint v2 = faces[j + 2];
            edge_map.insert(MapValueType(Edge(v0, v1), verts.size()));
            edge_map.insert(MapValueType(Edge(v1, v0), verts.size()));
            verts.push_back((verts[v0] + verts[v1]) / 2.0f);
            edge_map.insert(MapValueType(Edge(v1, v2), verts.size()));
            edge_map.insert(MapValueType(Edge(v2, v1), verts.size()));
            verts.push_back((verts[v1] + verts[v2]) / 2.0f);
            edge_map.insert(MapValueType(Edge(v2, v0), verts.size()));
            edge_map.insert(MapValueType(Edge(v0, v2), verts.size()));
            verts.push_back((verts[v2] + verts[v0]) / 2.0f);
        }

        /* Walk over faces and create new connectivity. */
        std::size_t num_faces = faces.size();
        for (std::size_t j = 0; j < num_faces; j += 3) {
            uint v0 = faces[j + 0];
            uint v1 = faces[j + 1];
            uint v2 = faces[j + 2];
            uint ev0 = edge_map[Edge(v0, v1)];
            uint ev1 = edge_map[Edge(v1, v2)];
            uint ev2 = edge_map[Edge(v2, v0)];
            faces.push_back(ev0); faces.push_back(v1); faces.push_back(ev1);
            faces.push_back(ev1); faces.push_back(v2); faces.push_back(ev2);
            faces.push_back(ev2); faces.push_back(v0); faces.push_back(ev0);
            faces[j + 0] = ev0;
            faces[j + 1] = ev1;
            faces[j + 2] = ev2;
        }
    }

    /* Normalize and transform vertices. */
    for (std::size_t i = 0; i < verts.size(); ++i) {
        verts[i].normalize();
        verts[i] = verts[i] * args.radius;
    }

    mve::TriangleMesh::Ptr mesh = mve::TriangleMesh::create();
    mesh->get_vertices().swap(verts);
    mesh->get_faces().swap(faces);

    mesh->recalc_normals();

    mve::geom::SavePLYOptions opts;
    opts.write_vertex_normals = true;
    mve::geom::save_ply_mesh(mesh, args.omesh, opts);

    return EXIT_SUCCESS;
}
