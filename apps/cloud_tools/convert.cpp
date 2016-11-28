#include <iostream>

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
};


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


Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_minnum(2);
    args.set_nonopt_maxnum(2);
    args.set_usage("Usage: " + std::string(argv[0]) + " [OPTS] IN_CLOUD OUT_CLOUD");
    args.add_option('b', "bounding-box", true, "Six comma separated values used as AABB.");
    args.set_description("Converts point cloud formats");
    args.parse(argc, argv);

    Arguments conf;
    conf.in_cloud = args.get_nth_nonopt(0);
    conf.out_cloud = args.get_nth_nonopt(1);

    for (util::ArgResult const* i = args.next_option();
         i != 0; i = args.next_option()) {
        switch (i->opt->sopt) {
        case 'b': conf.aabb = i->arg; break;
        default:
            throw std::invalid_argument("Invalid option");
        }
    }

    return conf;
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

    mve::geom::save_mesh(mesh, args.out_cloud);

    return EXIT_SUCCESS;
}
