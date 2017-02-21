#include <random>
#include <fstream>
#include <iostream>
#include <iterator>
#include <algorithm>

#include "math/octree_tools.h"

#include "util/system.h"
#include "util/arguments.h"
#include "util/tokenizer.h"

#include "mve/mesh_io.h"

#include "geom/cloud_io.h"

struct Arguments {
    std::string in_cloud;
    std::string out_cloud;
    std::string aabb;
    float fraction;
};

Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_minnum(2);
    args.set_nonopt_maxnum(2);
    args.set_usage("Usage: " + std::string(argv[0]) + " [OPTS] IN_CLOUD OUT_CLOUD");
    args.add_option('b', "bounding-box", true, "Six comma separated values used as AABB.");
    args.add_option('r', "reduce", true, "Reduce to fraction of vertices [1.0].");
    args.set_description("Converts point cloud formats");
    args.parse(argc, argv);

    Arguments conf;
    conf.in_cloud = args.get_nth_nonopt(0);
    conf.out_cloud = args.get_nth_nonopt(1);
    conf.fraction = 1.0f;

    for (util::ArgResult const* i = args.next_option();
         i != 0; i = args.next_option()) {
        switch (i->opt->sopt) {
        case 'b': conf.aabb = i->arg; break;
        case 'r': conf.fraction = i->get_arg<float>(); break;
        default:
            throw std::invalid_argument("Invalid option");
        }
    }

    return conf;
}

/* Derived from mve/apps/scene2pset/scene2pset.cc */
void
aabb_from_string (std::string const& str,
    math::Vec3f* aabb_min, math::Vec3f* aabb_max)
{
    util::Tokenizer tok;
    tok.split(str, ',');
    if (tok.size() != 6) {
        std::cerr << "Error: Invalid AABB given" << std::endl;
        std::exit(EXIT_FAILURE);
    }

    for (int i = 0; i < 3; ++i) {
        (*aabb_min)[i] = tok.get_as<float>(i);
        (*aabb_max)[i] = tok.get_as<float>(i + 3);
    }
    std::cout << "Using AABB: (" << (*aabb_min) << ") / ("
        << (*aabb_max) << ")" << std::endl;
}

template <typename T>
void remove_elements(std::vector<T> * vec, std::vector<bool> del)
{
    if (vec->empty()) return;

    typename std::vector<T>::iterator r, w;
    std::size_t i = 0;

    /* Advance till first item to delete. */
    while (!del[i]) ++i;

    r = w = vec->begin() + i;

    for (; i < vec->size(); ++i, ++r) {
        if (del[i]) continue;

        *w++ = *r;
    }

    vec->erase(w, vec->end());
}

int main(int argc, char **argv) {
    util::system::register_segfault_handler();
    util::system::print_build_timestamp(argv[0]);

    Arguments args = parse_args(argc, argv);

    math::Vec3f min, max;
    if (!args.aabb.empty()) {
        aabb_from_string(args.aabb, &min, &max);
    }

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

    if (args.fraction != 1.0f) {
        std::vector<math::Vec3f> const & verts = mesh->get_vertices();
        std::vector<std::size_t> idx(verts.size());
        std::iota(idx.begin(), idx.end(), 0);
        std::mt19937 g;
        std::shuffle(idx.begin(), idx.end(), g);

        std::size_t max_idx = verts.size() * args.fraction;

        std::vector<bool> del(verts.size(), false);
        for (std::size_t i = 0; i < verts.size(); ++i) {
            del[i] = idx[i] > max_idx;
        }

        remove_elements(&mesh->get_vertices(), del);
        remove_elements(&mesh->get_vertex_values(), del);
        remove_elements(&mesh->get_vertex_colors(), del);
        remove_elements(&mesh->get_vertex_confidences(), del);
    }

    if (!args.aabb.empty()) {
        std::vector<math::Vec3f> const & verts = mesh->get_vertices();
        std::vector<bool> del(verts.size(), false);

        for (std::size_t i = 0; i < verts.size(); ++i) {
            del[i] = !math::geom::point_box_overlap(verts[i], min, max);
        }

        remove_elements(&mesh->get_vertices(), del);
        remove_elements(&mesh->get_vertex_values(), del);
        remove_elements(&mesh->get_vertex_colors(), del);
        remove_elements(&mesh->get_vertex_confidences(), del);
    }

    if (args.out_cloud.size() >= 4
        && util::string::right(args.out_cloud, 4) == ".txt") {
        if (!mesh->has_vertex_values()) {
            std::cerr << "Cloud does not have vertex values." << std::endl;
            std::exit(EXIT_FAILURE);
        }

        std::ofstream out(args.out_cloud.c_str());
        if (!out.good()) {
            std::cerr << "Could not open file: "
                << std::strerror(errno) << std::endl;
            std::exit(EXIT_FAILURE);
        }
        std::vector<float> const & values = mesh->get_vertex_values();
        std::copy(values.begin(), values.end(), std::ostream_iterator<float>(out, "\n"));
        out.close();
    } else {
        mve::geom::save_mesh(mesh, args.out_cloud);
    }

    return EXIT_SUCCESS;
}
