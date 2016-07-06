#include <iostream>

#include "util/system.h"
#include "util/arguments.h"
#include "util/choices.h"

#include "mve/mesh_io_ply.h"
#include "acc/bvh_tree.h"

typedef unsigned int uint;

struct Arguments {
    std::string in_mesh;
    std::string gt_mesh;
    float comp_thresh;
    float acc_thresh;
    std::string accuracy_mesh;
    std::string completeness_mesh;
};

Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_minnum(2);
    args.set_nonopt_maxnum(2);
    args.set_usage("Usage: " + std::string(argv[0]) + " [OPTS] IN_MESH GT_MESH");
    args.set_description("Evaluate reconstruction with Middlebury mview methodology");
    args.add_option('a', "accuracy-mesh", true,
        "save in_mesh with distance as vertex value to give filename");
    args.add_option('c', "completeness-mesh", true,
        "save gt_mesh with distance as vertex value to given filename");
    args.add_option('\0', "accuracy-threshold", true, "percentile [90]");
    args.add_option('\0', "completeness-threshold", true, "distance [0.05]");
    args.parse(argc, argv);

    Arguments conf;
    conf.in_mesh = args.get_nth_nonopt(0);
    conf.gt_mesh = args.get_nth_nonopt(1);
    conf.comp_thresh = 0.05f;
    conf.acc_thresh = 90.0f;

    for (util::ArgResult const* i = args.next_option();
         i != 0; i = args.next_option()) {
        switch (i->opt->sopt) {
        case 'a':
            conf.accuracy_mesh = i->arg;
        break;
        case 'c':
            conf.completeness_mesh = i->arg;
        break;
        case '\0':
            if (i->opt->lopt == "accuracy-threshold") {
                conf.acc_thresh = i->get_arg<float>();
            } else if (i->opt->lopt == "completeness-threshold") {
                conf.comp_thresh = i->get_arg<float>();
            } else {
                throw std::invalid_argument("Invalid option");
            }
        default:
            throw std::invalid_argument("Invalid option");
        }
    }

    if (conf.acc_thresh < 0.0f || 100.0f < conf.acc_thresh) {
        throw std::invalid_argument("Accuracy threshold has to be in [0,100]");
    }

    return conf;
}

mve::TriangleMesh::Ptr
load_mesh(std::string const & path) {
    mve::TriangleMesh::Ptr mesh;
    try {
        mesh = mve::geom::load_ply_mesh(path);
    } catch (std::exception& e) {
        std::cerr << "\tCould not load mesh: "<< e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }
    return mesh;
}

acc::BVHTree<uint, math::Vec3f>::Ptr
create_bvh_tree(mve::TriangleMesh::Ptr mesh) {
    std::vector<math::Vec3f> const & vertices = mesh->get_vertices();
    std::vector<uint> const & faces = mesh->get_faces();
    return acc::BVHTree<uint, math::Vec3f>::create(faces, vertices);
}

float
calculate_accuracy(mve::TriangleMesh::Ptr in_mesh,
    mve::TriangleMesh::Ptr gt_mesh, float threshold)
{
    std::vector<math::Vec3f> const & verts = in_mesh->get_vertices();
    std::vector<float> & values = in_mesh->get_vertex_values();
    acc::BVHTree<uint, math::Vec3f>::Ptr bvh_tree;
    bvh_tree = create_bvh_tree(gt_mesh);

    gt_mesh->ensure_normals(true, false);
    std::vector<math::Vec3f> const & gt_face_normals = gt_mesh->get_face_normals();

    values.resize(verts.size());
    #pragma omp parallel for
    for (std::size_t i = 0; i < verts.size(); ++i) {
        math::Vec3f q = verts[i];
        math::Vec3f p = bvh_tree->closest_point(q);
        math::Vec3f qp = (p - q);
        float dist = qp.norm();

        /* Determine sign if distance is large enough. */
        if (dist > 1e-7f) {
            acc::BVHTree<uint, math::Vec3f>::Ray ray;
            ray.origin = q;
            ray.dir = qp / dist;
            ray.tmin = 0.0f;
            ray.tmax = dist + 1e-3f;
            acc::BVHTree<uint, math::Vec3f>::Hit hit;

            if (bvh_tree->intersect(ray, &hit)) {
                float cosine = ray.dir.dot(gt_face_normals[hit.idx]);
                if (cosine > 0.0f) continue;

                dist *= -1.0f;
            }
        }

        values[i] = dist;
    }

    std::vector<float> dists(values.size());
    for (std::size_t i = 0; i < dists.size(); ++i) {
        dists[i] = std::abs(values[i]);
    }
    std::sort(dists.begin(), dists.end());

    return dists[dists.size() * (threshold / 100.0f)];
}

float
calculate_completeness(mve::TriangleMesh::Ptr in_mesh,
    mve::TriangleMesh::Ptr gt_mesh, float threshold)
{
    std::vector<math::Vec3f> const & verts = gt_mesh->get_vertices();
    std::vector<float> & values = gt_mesh->get_vertex_values();
    acc::BVHTree<uint, math::Vec3f>::Ptr bvh_tree;
    bvh_tree = create_bvh_tree(in_mesh);

    values.resize(verts.size());
    #pragma omp parallel for
    for (std::size_t i = 0; i < verts.size(); ++i) {
        math::Vec3f q = verts[i];
        math::Vec3f p = bvh_tree->closest_point(q);
        float dist = (q - p).norm();
        values[i] = dist;
    }

    uint covered = 0;
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (values[i] > threshold) continue;
        covered += 1;
    }

    return covered / static_cast<float>(values.size());
}

int main(int argc, char **argv) {
    util::system::register_segfault_handler();
    util::system::print_build_timestamp(argv[0]);

    Arguments args = parse_args(argc, argv);

    mve::TriangleMesh::Ptr in_mesh, gt_mesh;
    in_mesh = load_mesh(args.in_mesh);
    gt_mesh = load_mesh(args.gt_mesh);

    float acc = calculate_accuracy(in_mesh, gt_mesh, args.acc_thresh);
    std::cout << "Accuracy: " << acc << std::endl;
    float comp = calculate_completeness(in_mesh, gt_mesh, args.comp_thresh);
    std::cout << "Completeness: " << comp << std::endl;

    mve::geom::SavePLYOptions opts;
    opts.write_vertex_values = true;
    if (!args.accuracy_mesh.empty()) {
        mve::geom::save_ply_mesh(in_mesh, args.accuracy_mesh, opts);
    }
    if (!args.completeness_mesh.empty()) {
        mve::geom::save_ply_mesh(gt_mesh, args.completeness_mesh, opts);
    }

    return EXIT_SUCCESS;
}
