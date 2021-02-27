/*
 * Copyright (C) 2016-2018, Nils Moehrle
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include <random>

#include <iostream>

#include "util/system.h"
#include "util/arguments.h"
#include "util/tokenizer.h"

#include "math/geometry.h"

#include "mve/mesh_io_ply.h"
#include "acc/bvh_tree.h"

typedef unsigned int uint;

struct Range {
    float min;
    float step;
    float max;
};

std::vector<float>
expand_range(Range const & range) {
    std::vector<float> ret;
    for (float x = range.min; x < range.max; x += range.step) {
        ret.push_back(x);
    }
    return ret;
}

Range parse_range(std::string const & str) {
    util::Tokenizer tok;
    tok.split(str, ':');

    if (tok.size() != 3) throw std::runtime_error("Invalid range");

    Range ret = {tok.get_as<float>(0), tok.get_as<float>(1), tok.get_as<float>(2)};

    if (ret.step == 0.0f
        || (ret.min > ret.max && ret.step > 0.0f)
        || (ret.min < ret.max && ret.step < 0.0f)) {
        throw std::runtime_error("Invalid range");
    }

    return ret;
}

struct Arguments {
    std::string in_mesh;
    std::string gt_mesh;
    Range comp_range;
    Range acc_range;
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
    //args.add_option('c', "completeness-mesh", true,
    //    "save gt_mesh with distance as vertex value to given filename");
    args.add_option('\0', "accuracy-range", true, "percentile [0.8:0.01:0.99]");
    args.add_option('\0', "completeness-range", true, "distance [0.005:0.005:0.01]");
    args.parse(argc, argv);

    Arguments conf;
    conf.in_mesh = args.get_nth_nonopt(0);
    conf.gt_mesh = args.get_nth_nonopt(1);
    conf.acc_range = {0.8f, 0.01f, 0.99f};
    conf.comp_range = {0.005f, 0.005f, 0.1f};

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
                conf.acc_range = parse_range(i->arg);
            } else if (i->opt->lopt == "completeness-threshold") {
                conf.comp_range = parse_range(i->arg);
            } else {
                throw std::invalid_argument("Invalid option");
            }
        default:
            throw std::invalid_argument("Invalid option");
        }
    }

    if (conf.acc_range.min < 0.0f || 1.0f < conf.acc_range.max) {
        throw std::invalid_argument("Accuracy range has to be in [0,1]");
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

void
calculate_accuracy(mve::TriangleMesh::Ptr in_mesh,
    mve::TriangleMesh::Ptr gt_mesh, Range acc_range)
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
                if (cosine < 0.0f) {
                    dist *= -1.0f;
                }
            }
        }

        values[i] = dist;
    }

    std::vector<float> dists(values.size());
    for (std::size_t i = 0; i < dists.size(); ++i) {
        dists[i] = std::abs(values[i]);
    }
    std::sort(dists.begin(), dists.end());

    std::vector<float> threshs = expand_range(acc_range);
    std::cout << "Accuracy:" << std::endl;
    for (float threshold : threshs) {
        float acc = dists[dists.size() * threshold];
        std::cout << threshold << ' ' << acc << std::endl;
    }
}

void
calculate_completeness(mve::TriangleMesh::Ptr in_mesh,
    mve::TriangleMesh::Ptr gt_mesh, Range comp_range)
{
    std::vector<math::Vec3f> const & verts = gt_mesh->get_vertices();
    std::vector<float> values; // = gt_mesh->get_vertex_values(); //TODO fix
    acc::BVHTree<uint, math::Vec3f>::Ptr bvh_tree;
    bvh_tree = create_bvh_tree(in_mesh);

    std::vector<uint> const & faces = gt_mesh->get_faces();

    uint num_in_verts = in_mesh->get_vertices().size();

    std::uniform_real_distribution<float> dist(0.0f, 1.0f);
    std::atomic<int> num_threads(0);

    float surface = 0.0f;
    std::vector<math::Vec3f> samples;

    /* Sample surface to obtain similar number of vertices as in_mesh. */
    #pragma omp parallel
    {
        num_threads += 1;
        std::mt19937 gen;

        #pragma omp for reduction(+:surface)
        for (std::size_t i = 0; i < faces.size(); i += 3) {
            math::Vec3f v0 = verts[faces[i + 0]];
            math::Vec3f v1 = verts[faces[i + 1]];
            math::Vec3f v2 = verts[faces[i + 2]];

            surface += math::geom::triangle_area(v0, v1, v2);
        }

        float area_per_sample = surface / (10 * num_in_verts);

        std::vector<math::Vec3f> tsamples;
        tsamples.reserve(num_in_verts / num_threads);

        #pragma omp for schedule(static)
        for (std::size_t i = 0; i < faces.size(); i += 3) {
            gen.seed(i);

            math::Vec3f v0 = verts[faces[i + 0]];
            math::Vec3f v1 = verts[faces[i + 1]];
            math::Vec3f v2 = verts[faces[i + 2]];

            float area = math::geom::triangle_area(v0, v1, v2);

            float face_samples = area / area_per_sample;
            uint num_face_samples = face_samples;

            if (dist(gen) < (face_samples - static_cast<float>(num_face_samples))) {
                num_face_samples += 1;
            }

            for (uint j = 0; j < num_face_samples; ++j) {
                float r1 = dist(gen);
                float r2 = dist(gen);

                float tmp = std::sqrt(r1);
                float u = 1.0f - tmp;
                float v = r2 * tmp;

                float w = 1.0f - v - u;

                tsamples.push_back(u * v0 + v * v1 + w * v2);
            }
        }

        #pragma omp critical
        {
            samples.insert(samples.end(), tsamples.begin(), tsamples.end());
        }
    }

    values.resize(samples.size());
    #pragma omp parallel for
    for (std::size_t i = 0; i < samples.size(); ++i) {
        math::Vec3f q = samples[i];
        math::Vec3f p = bvh_tree->closest_point(q);
        float dist = (q - p).norm();
        values[i] = dist;
    }


    mve::TriangleMesh::Ptr cloud = mve::TriangleMesh::create();
    cloud->get_vertices().assign(samples.begin(), samples.end());
    cloud->get_vertex_values().assign(values.begin(), values.end());
    mve::geom::SavePLYOptions opts;
    opts.write_vertex_confidences = false;
    opts.write_vertex_colors = false;
    opts.write_vertex_values = true;
    //mve::geom::save_ply_mesh(cloud, "/tmp/cloud.ply", opts);

    std::vector<float> threshs = expand_range(comp_range);
    std::cout << "Completeness: " << std::endl;
    for (float threshold : threshs) {
        std::size_t covered = 0;
        for (std::size_t i = 0; i < values.size(); ++i) {
            if (values[i] < threshold) {
                covered += 1;
            }
        }
        float comp = covered / static_cast<float>(values.size());
        std::cout << threshold << ' ' << comp << std::endl;
    }
}

int main(int argc, char **argv) {
    util::system::register_segfault_handler();
    util::system::print_build_timestamp(argv[0]);

    Arguments args = parse_args(argc, argv);

    mve::TriangleMesh::Ptr in_mesh, gt_mesh;
    in_mesh = load_mesh(args.in_mesh);
    gt_mesh = load_mesh(args.gt_mesh);

    mve::geom::SavePLYOptions opts;
    opts.write_vertex_confidences = false;
    opts.write_vertex_colors = false;
    opts.write_vertex_values = true;

    calculate_accuracy(in_mesh, gt_mesh, args.acc_range);
    if (!args.accuracy_mesh.empty()) {
        mve::geom::save_ply_mesh(in_mesh, args.accuracy_mesh, opts);
    }

    calculate_completeness(in_mesh, gt_mesh, args.comp_range);
    if (!args.completeness_mesh.empty()) {
        mve::geom::save_ply_mesh(gt_mesh, args.completeness_mesh, opts);
    }

    return EXIT_SUCCESS;
}
