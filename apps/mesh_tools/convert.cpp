#include <iostream>
#include <vector>

#include "util/arguments.h"

#include "util/matrix_io.h"

#include "mve/mesh_tools.h"
#include "mve/mesh_io_ply.h"

#include "acc/primitives.h"

#include "geom/transform.h"

typedef unsigned int uint;

constexpr float inf = std::numeric_limits<float>::infinity();

struct Arguments {
    std::string in_mesh;
    std::string out_mesh;
    std::string transform;
    mve::geom::SavePLYOptions opts;
    bool invert;
    bool clean;
    bool show_info;
    bool delete_faces;
};

Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_maxnum(2);
    args.set_nonopt_minnum(2);
    args.set_usage("Usage: " + std::string(argv[0]) + " [OPTS] IN_MESH OUT_MESH");
    args.set_description("Conversion app for meshes a la image magicks convert");
    args.add_option('t', "transform", true, "transform vertices with matrix file");
    args.add_option('i', "invert", false, "invert transform");
    args.add_option('c', "clean", false, "cleanup mesh");
    args.add_option('\0', "show-info", false, "show info");
    args.add_option('\0', "ascii", false, "write out ascii file");
    args.add_option('\0', "delete-faces", false, "delete faces");
    args.add_option('\0', "delete-vnormals", false, "delete vertex normals");
    args.add_option('\0', "delete-vcolors", false, "delete vertex colors");
    args.add_option('\0', "delete-values", false, "delete vertex values");
    args.add_option('\0', "delete-confidences", false, "delete vertex confidences");
    args.parse(argc, argv);

    Arguments conf;
    conf.in_mesh = args.get_nth_nonopt(0);
    conf.out_mesh = args.get_nth_nonopt(1);
    conf.invert = false;
    conf.clean = false;
    conf.show_info = false;
    conf.delete_faces = false;

    conf.opts.format_binary = true;
    conf.opts.write_face_colors = false;
    conf.opts.write_face_normals = false;
    conf.opts.write_vertex_colors = true;
    conf.opts.write_vertex_confidences = true;
    conf.opts.write_vertex_values = true;
    conf.opts.write_vertex_normals = true;

    for (util::ArgResult const* i = args.next_option();
         i != nullptr; i = args.next_option()) {
        switch (i->opt->sopt) {
        case 't':
            conf.transform = i->arg;
        break;
        case 'i':
            conf.invert = true;
        break;
        case 'c':
            conf.clean = true;
        break;
        case '\0':
            if (i->opt->lopt == "ascii") {
                conf.opts.format_binary = false;
            } else if (i->opt->lopt == "show-info") {
                conf.show_info = true;
            } else if (i->opt->lopt == "delete-faces") {
                conf.delete_faces = true;
            } else if (i->opt->lopt == "delete-vnormals") {
                conf.opts.write_vertex_normals = false;
            } else if (i->opt->lopt == "delete-vcolors") {
                conf.opts.write_vertex_colors = false;
            } else if (i->opt->lopt == "delete-values") {
                conf.opts.write_vertex_values = false;
            } else if (i->opt->lopt == "delete-confidences") {
                conf.opts.write_vertex_confidences = false;
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
    Arguments args = parse_args(argc, argv);

    mve::TriangleMesh::Ptr mesh;
    try {
        mesh = mve::geom::load_ply_mesh(args.in_mesh);
    } catch (std::exception& e) {
        std::cerr << "\tCould not load mesh: "<< e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }
    mesh->ensure_normals(true, false);

    std::vector<uint> & faces = mesh->get_faces();
    if (args.clean) {
        std::vector<math::Vec3f> const & normals = mesh->get_face_normals();
        std::vector<uint> tmp(faces.size());

        std::size_t w = 0;
        for (std::size_t i = 0; i < normals.size(); ++i) {
            math::Vec3f const & normal = normals[i];
            if (normal.norm() > 0.0f) {
                tmp[w++] = faces[3 * i + 0];
                tmp[w++] = faces[3 * i + 1];
                tmp[w++] = faces[3 * i + 2];
            }
        }
        tmp.resize(w);

        std::size_t num_del_faces = faces.size() - tmp.size();
        std::swap(faces, tmp);

        std::size_t num_del_verts = mve::geom::mesh_delete_unreferenced(mesh);
        std::cout << "Deleted" << num_del_faces << " faces and "
            << num_del_verts << " vertices" << std::endl;
        mesh->ensure_normals(true, false);
    }

    std::vector<math::Vec3f> & vertices = mesh->get_vertices();
    std::vector<math::Vec3f> & normals = mesh->get_vertex_normals();

    if (args.show_info) {
        acc::AABB<math::Vec3f> aabb = acc::calculate_aabb(vertices);

        std::cout << aabb.min << " " << aabb.max << std::endl;
    }

    if (args.delete_faces) {
        faces.clear();
    }

    if (!args.transform.empty()) {
        math::Matrix4f T = load_matrix_from_file(args.transform);

        if (args.invert) {
            T = inverse_transform(T);
        }

        for (std::size_t i = 0; i < vertices.size(); ++i) {
            vertices[i] = T.mult(vertices[i], 1.0f);
        }

        for (std::size_t i = 0; i < normals.size(); ++i) {
            normals[i] = T.mult(normals[i], 0.0f).normalize();
        }
    }

    mve::geom::save_ply_mesh(mesh, args.out_mesh, args.opts);
}
