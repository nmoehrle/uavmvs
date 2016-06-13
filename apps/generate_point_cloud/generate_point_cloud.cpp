#include <iostream>
#include <vector>
#include <random>
#include <deque>

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

    std::cout << "Load mesh: " << std::endl;
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

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dist(0.0f, 1.0f);

    #pragma omp parallel
    {
        std::vector<math::Vec3f> samples;

        #pragma omp for
        for (std::size_t i = 0; i < faces.size(); i += 3) {
            math::Vec3f v0 = verts[faces[i + 0]];
            math::Vec3f v1 = verts[faces[i + 1]];
            math::Vec3f v2 = verts[faces[i + 2]];

            float area = math::geom::triangle_area(v0, v1, v2);

            uint num_samples = std::ceil(area * args.samples);

            samples.reserve(samples.size() + num_samples);
            for (uint j = 0; j < num_samples; ++j) {
                float u = dist(gen);
                float v = dist(gen);

                float tmp = std::sqrt(u);
                u = 1.0f - tmp;
                v = tmp * v;

                float w = 1.0f - v - u;

                samples.push_back(u * v0 + v * v1 + w * v2);
            }
        }

        #pragma omp critical
        {
            overts.insert(overts.end(), samples.begin(), samples.end());
        }
    }
    mve::geom::save_ply_mesh(omesh, args.out_cloud);
}
