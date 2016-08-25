#include <iostream>
#include <fstream>
#include <vector>

#include "util/arguments.h"
#include "mve/mesh_io_ply.h"
#include "mve/bundle_io.h"

typedef unsigned int uint;

constexpr float inf = std::numeric_limits<float>::infinity();

struct Arguments {
    std::string in_mesh;
    std::string out_mesh;
    std::string transform;
    mve::geom::SavePLYOptions opts;
    bool invert;
    bool show_info;
    bool flip_normals;
};

Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_maxnum(2);
    args.set_nonopt_minnum(2);
    args.set_usage("Usage: " + std::string(argv[0]) + " [OPTS] IN_MESH OUT_MESH");
    args.set_description("Prepare mesh for ...");
    args.add_option('t', "transform", true, "transform vertices with matrix file");
    args.add_option('i', "invert", false, "invert transform");
    args.add_option('s', "show-info", false, "show info");
    args.add_option('f', "flip-normals", false, "flip vertex normals");
    args.add_option('\0', "ascii", false, "write out ascii file");
    args.parse(argc, argv);

    Arguments conf;
    conf.in_mesh = args.get_nth_nonopt(0);
    conf.out_mesh = args.get_nth_nonopt(1);
    conf.invert = false;
    conf.show_info = false;
    conf.flip_normals = false;

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
        case 's':
            conf.show_info = true;
        break;
        case 'f':
            conf.flip_normals = true;
        break;
        case '\0':
            if (i->opt->lopt == "ascii") {
                conf.opts.format_binary = false;
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

math::Matrix4f
load_matrix_from_file(std::string const & filename) {
    math::Matrix4f ret;
    std::ifstream in(filename.c_str());
    if (!in.good()) {
        throw std::runtime_error("Could not open matrix file");
    }

    for (int i = 0; i < 16; ++i) {
        in >> ret[i];
    }

    if (in.fail()) {
        in.close();
        throw std::runtime_error("Invalid matrix file");
    }

    in.close();

    return ret;
}

#define CONVERT_TO_BUNDLE 0
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

    uint const num_faces = mesh->get_faces().size() / 3;
    std::vector<math::Vec3f> & vertices = mesh->get_vertices();
    std::vector<math::Vec3f> & normals = mesh->get_vertex_normals();
    std::vector<uint> & faces = mesh->get_faces();
    std::vector<math::Vec4f> & colors = mesh->get_vertex_colors();

    if (args.show_info) {
        math::Vec3f min(+inf);
        math::Vec3f max(-inf);

        for (std::size_t i = 0; i < vertices.size(); ++i) {
            for (std::size_t j = 0; j < 3; ++j){
                min[j] = std::min(min[j], vertices[i][j]);
                max[j] = std::max(max[j], vertices[i][j]);
            }
        }

        std::cout << min << " " << max << std::endl;
    }

    if (!args.transform.empty()) {
        math::Matrix4f T = load_matrix_from_file(args.transform);

        if (args.invert) {
            math::Matrix3f R;
            R(0, 0) = T(0, 0); R(0, 1) = T(1, 0); R(0, 2) = T(2, 0);
            R(1, 0) = T(0, 1); R(1, 1) = T(1, 1); R(1, 2) = T(2, 1);
            R(2, 0) = T(0, 2); R(2, 1) = T(1, 2); R(2, 2) = T(2, 2);
            math::Vec3f t = -R * math::Vec3f(T(0, 3), T(1, 3), T(2, 3));
            T = R.hstack(t).vstack(math::Vec4f(0.0f, 0.0f, 0.0f, 1.0f));
        }


        for (std::size_t i = 0; i < vertices.size(); ++i) {
            vertices[i] = T.mult(vertices[i], 1.0f);
        }

        for (std::size_t i = 0; i < normals.size(); ++i) {
            normals[i] = T.mult(normals[i], 0.0f);
        }
    }

    if (args.flip_normals) {
        for (std::size_t i = 0; i < normals.size(); ++i) {
            normals[i] = -normals[i];
        }
    }

#if CONVERT_TO_BUNDLE
    mve::Bundle::Ptr bundle = mve::Bundle::create();
    std::vector<mve::Bundle::Feature3D> & features = bundle->get_features();
    features.resize(vertices.size());

    for (std::size_t i = 0; i < vertices.size(); ++i) {
        mve::Bundle::Feature3D & feature = features[i];
        math::Vec3f const & vertex = vertices[i];
        math::Vec4f const & color = colors[i];
        std::copy(vertex.begin(), vertex.end(), feature.pos);
        std::copy(color.begin(), color.end() - 1, feature.color);
    }

    mve::save_mve_bundle(bundle, args.out_mesh);
#endif

    mve::geom::save_ply_mesh(mesh, args.out_mesh, args.opts);
}
