/*
 * Copyright (C) 2016-2018, Nils Moehrle
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include <vector>
#include <atomic>
#include <random>
#include <iostream>
#include <algorithm>

#include "math/geometry.h"

#include "util/arguments.h"

#include "mve/mesh.h"
#include "mve/mesh_io_ply.h"

#include "acc/math.h"
#include "acc/primitives.h"

typedef unsigned int uint;

struct Arguments {
    std::string in_mesh;
    std::string out_cloud;
    uint samples;
};

Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_maxnum(2);
    args.set_nonopt_minnum(2);
    args.set_usage("Usage: " + std::string(argv[0]) + " [OPTS] IN_MESH OUT_CLOUD");
    args.set_description("Create point cloud by uniformly sampling the mesh surface");
    args.add_option('s', "samples", true, "point samples per unit square [100]");
    args.parse(argc, argv);

    Arguments conf;
    conf.in_mesh = args.get_nth_nonopt(0);
    conf.out_cloud = args.get_nth_nonopt(1);
    conf.samples = 100;

    for (util::ArgResult const* i = args.next_option();
         i != nullptr; i = args.next_option()) {
        switch (i->opt->sopt) {
        case 's':
            conf.samples = i->get_arg<uint>();
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

    std::vector<math::Vec3f> const & verts = mesh->get_vertices();
    std::vector<uint> const & faces = mesh->get_faces();

    mve::TriangleMesh::Ptr omesh = mve::TriangleMesh::create();
    std::vector<math::Vec3f> & overts = omesh->get_vertices();
    std::vector<math::Vec3f> & onormals = omesh->get_vertex_normals();

    std::uniform_real_distribution<float> dist(0.0f, 1.0f);
    std::atomic<int> num_threads(0);

    float surface = 0.0f;
    #pragma omp parallel
    {
        num_threads += 1;
        std::mt19937 gen;
        std::vector<math::Vec3f> samples;
        std::vector<math::Vec3f> normals;

        #pragma omp for reduction(+:surface)
        for (std::size_t i = 0; i < faces.size(); i += 3) {
            math::Vec3f v0 = verts[faces[i + 0]];
            math::Vec3f v1 = verts[faces[i + 1]];
            math::Vec3f v2 = verts[faces[i + 2]];

            surface += math::geom::triangle_area(v0, v1, v2);
        }
        std::size_t num_samples = surface * args.samples;

        samples.reserve(num_samples / num_threads);
        normals.reserve(num_samples / num_threads);

        #pragma omp for schedule(static)
        for (std::size_t i = 0; i < faces.size(); i += 3) {
            gen.seed(i);

            math::Vec3f v0 = verts[faces[i + 0]];
            math::Vec3f v1 = verts[faces[i + 1]];
            math::Vec3f v2 = verts[faces[i + 2]];
            math::Vec3f normal = (v1 - v0).cross(v2 - v0).normalized();

            float area = math::geom::triangle_area(v0, v1, v2);

            float face_samples = area * args.samples;
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

                samples.push_back(u * v0 + v * v1 + w * v2);
                normals.push_back(normal);
            }
        }

        #pragma omp critical
        {
            overts.insert(overts.end(), samples.begin(), samples.end());
            onormals.insert(onormals.end(), normals.begin(), normals.end());
        }
    }

    std::vector<std::size_t> indices(overts.size());
    std::vector<std::uint64_t> zindices(overts.size());

    {
        acc::AABB<math::Vec3f> aabb = acc::calculate_aabb(overts);
        math::Vec3d scale, bias;
        for (int i = 0; i < 3; ++i) {
            double div = (aabb.max[i] - aabb.min[i]);
            scale[i] = 1.0 / div * (double(1 << 20) - 1.0);
            bias[i] = - aabb.min[i] / div * (double(1 << 20) - 1.0);
        }
        for (std::size_t i = 0; i < zindices.size(); ++i) {
            math::Vec3f const & vert = overts[i];
            math::Vector<std::uint32_t, 3> position;
            for (int j = 0; j < 3; ++j) {
                position[j] = scale[j] * vert[j] + bias[j];
            }
            zindices[i] = acc::z_order_index(position);
        }
    }

    std::iota(indices.begin(), indices.end(), 0);
    std::sort(indices.begin(), indices.end(),
        [&zindices, &overts] (uint l, uint r) -> bool {
            return zindices[l] < zindices[r]
                || (zindices[l] == zindices[r]
                    && std::lexicographical_compare(
                        overts[l].begin(), overts[l].end(),
                        overts[r].begin(), overts[r].end()));
        }
    );

    std::vector<math::Vec3f> tmp(overts.size());
    for (std::size_t i = 0; i < indices.size(); ++i) {
        tmp[i] = overts[indices[i]];
    }
    std::swap(tmp, overts);
    for (std::size_t i = 0; i < indices.size(); ++i) {
        tmp[i] = onormals[indices[i]];
    }
    std::swap(tmp, onormals);

#if IDX2VALUE
    std::vector<float> & ovalues = omesh->get_vertex_values();
    ovalues.resize(overts.size());
    for (std::size_t i = 0; i < overts.size(); ++i) {
        ovalues[i] = static_cast<float>(i) / overts.size();
    }
#endif

    mve::geom::SavePLYOptions opts;
    opts.write_vertex_normals = true;
    mve::geom::save_ply_mesh(omesh, args.out_cloud, opts);
}
