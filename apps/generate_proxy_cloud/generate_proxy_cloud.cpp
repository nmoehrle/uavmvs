#include <iostream>
#include <vector>
#include <atomic>
#include <random>

#include "math/geometry.h"

#include "util/arguments.h"
#include "mve/mesh.h"
#include "mve/mesh_io_ply.h"

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

    #pragma omp parallel
    {
        num_threads += 1;
        std::mt19937 gen;
        std::vector<math::Vec3f> samples;
        std::vector<math::Vec3f> normals;

        #pragma omp for schedule(static)
        for (std::size_t i = 0; i < faces.size(); i += 3) {
            gen.seed(i);

            math::Vec3f v0 = verts[faces[i + 0]];
            math::Vec3f v1 = verts[faces[i + 1]];
            math::Vec3f v2 = verts[faces[i + 2]];
            math::Vec3f normal = (v1 - v0).cross(v2 - v0).normalized();

            float area = math::geom::triangle_area(v0, v1, v2);

            uint num_samples = std::ceil(area * args.samples);

            samples.reserve(samples.size() + num_samples);
            normals.reserve(normals.size() + num_samples);
            for (uint j = 0; j < num_samples; ++j) {
                float u = dist(gen);
                float v = dist(gen);

                float tmp = std::sqrt(u);
                u = 1.0f - tmp;
                v = tmp * v;

                float w = 1.0f - v - u;

                samples.push_back(u * v0 + v * v1 + w * v2);
                normals.push_back(normal);
            }
        }

        #pragma omp for ordered
        for (int i = 0; i < num_threads; ++i) {
            #pragma omp ordered
            {
                overts.insert(overts.end(), samples.begin(), samples.end());
                onormals.insert(onormals.end(), normals.begin(), normals.end());
            }
        }
    }

    std::vector<std::size_t> indices(overts.size());
    std::iota(indices.begin(), indices.end(), 0);
    std::sort(indices.begin(), indices.end(),
        [&overts] (uint l, uint r) -> bool {
            return std::lexicographical_compare(&overts[l][0], &overts[l][3], &overts[r][0], &overts[r][3]);
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

    mve::geom::SavePLYOptions opts;
    opts.write_vertex_normals = true;
    mve::geom::save_ply_mesh(omesh, args.out_cloud, opts);
}
