#include <cassert>
#include <iostream>

#include "util/system.h"
#include "util/arguments.h"
#include "util/file_system.h"

#include "util/io.h"

#include "mve/mesh_io_ply.h"
#include "mve/image_io.h"
#include "mve/image_tools.h"

#include "acc/primitives.h"

#include "cacc/math.h"
#include "cacc/util.h"
#include "cacc/bvh_tree.h"
#include "cacc/tracing.h"
#include "cacc/point_cloud.h"

#include "eval/kernels.h"

constexpr float lowest = std::numeric_limits<float>::lowest();

struct Arguments {
    std::string proxy_mesh;
    std::string proxy_cloud;
    std::string airspace_mesh;
    std::string ovolume;
    float resolution;
    float max_distance;
    float max_altitude;
};

Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_minnum(4);
    args.set_nonopt_maxnum(4);
    args.set_usage("Usage: " + std::string(argv[0]) + " [OPTS] PROXY_MESH PROXY_CLOUD AIRSPACE_MESH OUT_VOLUME");
    args.set_description("TODO");
    args.add_option('r', "resolution", true, "guidance volume resolution [1.0]");
    args.add_option('\0', "max-distance", true, "maximum distance to surface [80.0]");
    args.add_option('\0', "max-altitude", true, "maximum altitude [100.0]");
    args.add_option('c', "cloud", true, "save cloud as ply file");
    args.parse(argc, argv);

    Arguments conf;
    conf.proxy_mesh = args.get_nth_nonopt(0);
    conf.proxy_cloud = args.get_nth_nonopt(1);
    conf.airspace_mesh = args.get_nth_nonopt(2);
    conf.ovolume = args.get_nth_nonopt(3);
    conf.resolution = 1.0f;
    conf.max_distance = 80.0f;
    conf.max_altitude = 100.0f;

    for (util::ArgResult const* i = args.next_option();
         i != 0; i = args.next_option()) {
        switch (i->opt->sopt) {
        case 'r':
            conf.resolution = i->get_arg<float>();
        break;
        case '\0':
            if (i->opt->lopt == "max-distance") {
                conf.max_distance = i->get_arg<float>();
            } else if (i->opt->lopt == "max-altitude") {
                conf.max_altitude = i->get_arg<float>();
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
    util::system::register_segfault_handler();
    util::system::print_build_timestamp(argv[0]);

    Arguments args = parse_args(argc, argv);

    cacc::select_cuda_device(3, 5);

    cacc::BVHTree<cacc::DEVICE>::Ptr dbvh_tree;
    {
        acc::BVHTree<uint, math::Vec3f>::Ptr bvh_tree;
        bvh_tree = load_mesh_as_bvh_tree(args.proxy_mesh);
        dbvh_tree = cacc::BVHTree<cacc::DEVICE>::create<uint, math::Vec3f>(bvh_tree);
    }
    cacc::tracing::bind_textures(dbvh_tree->cdata());

    mve::TriangleMesh::Ptr mesh;
    try {
        mesh = mve::geom::load_ply_mesh(args.airspace_mesh);
    } catch (std::exception& e) {
        std::cerr << "\tCould not load mesh: " << e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }

    std::vector<math::Vec3f> const & verts = mesh->get_vertices();

    //TODO merge with proxy mesh generation code
    acc::AABB<math::Vec3f> aabb = acc::calculate_aabb(verts);

    assert(acc::valid(aabb) && acc::volume(aabb) > 0.0f);

    int width = (aabb.max[0] - aabb.min[0]) / args.resolution + 1.0f;
    int height = (aabb.max[1] - aabb.min[1]) / args.resolution + 1.0f;
    int depth = args.max_altitude / args.resolution + 1.0f;

    std::cout << width << "x" << height << "x" << depth << std::endl;

    /* Create height map. */
    mve::FloatImage::Ptr hmap = mve::FloatImage::create(width, height, 1);
    hmap->fill(lowest);
    for (std::size_t i = 0; i < verts.size(); ++i) {
        math::Vec3f vertex = verts[i];
        int x = (vertex[0] - aabb.min[0]) / args.resolution + args.resolution / 2.0f;
        assert(0 <= x && x < width);
        int y = (vertex[1] - aabb.min[1]) / args.resolution + args.resolution / 2.0f;
        assert(0 <= y && y < height);
        float height = vertex[2];
        float z = hmap->at(x, y, 0);
        if (z > height) continue;

        hmap->at(x, y, 0) = height;
    }

    /* Estimate ground level and normalize height map */
    float ground_level = std::numeric_limits<float>::max();
    #pragma omp parallel for reduction(min:ground_level)
    for (int i = 0; i < hmap->get_value_amount(); ++i) {
        float height = hmap->at(i);
        if (height != lowest && height < ground_level) {
            ground_level = height;
        }
    }

    #pragma omp parallel for
    for (int i = 0; i < hmap->get_value_amount(); ++i) {
        float height = hmap->at(i);
        hmap->at(i) = (height != lowest) ? height - ground_level : 0.0f;
    }
    //ODOT merge with proxy mesh generation code

    mve::TriangleMesh::Ptr ocloud = mve::TriangleMesh::create();
    std::vector<math::Vec3f> & overts = ocloud->get_vertices();
    overts.resize(width * height * depth);
    std::vector<float> & ovalues = ocloud->get_vertex_values();
    ovalues.resize(width * height * depth, 0.0f);

    uint num_samples = 0;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {

            float px = (x - args.resolution / 2.0f) * args.resolution + aabb.min[0];
            float py = (y - args.resolution / 2.0f) * args.resolution + aabb.min[1];

            float fz = hmap->at(x, y, 0);

            for (int z = 0; z < depth; ++z) {
                float pz = ground_level + z * args.resolution;

                int idx = (z * height + y) * width + x;

                overts[idx] = math::Vec3f(px, py, pz);
                if (pz > fz) {
                    num_samples += 1;
                } else {
                    ovalues[idx] = -1.0f;
                }
            }
        }
    }

    cacc::PointCloud<cacc::DEVICE>::Ptr dcloud;
    {
        cacc::PointCloud<cacc::HOST>::Ptr cloud;
        cloud = load_point_cloud(args.proxy_cloud);
        dcloud = cacc::PointCloud<cacc::DEVICE>::create<cacc::HOST>(cloud);
    }

    cacc::PointCloud<cacc::HOST>::Ptr hvolume;
    cacc::PointCloud<cacc::DEVICE>::Ptr dvolume;
    {
        hvolume = cacc::PointCloud<cacc::HOST>::create(num_samples);
        cacc::PointCloud<cacc::HOST>::Data data = hvolume->cdata();
        for (std::size_t i = 0, it = 0; i < overts.size(); ++i) {
            if (ovalues[i] == -1.0f) continue;

            data.vertices_ptr[it] = cacc::Vec3f(overts[i].begin());
            data.normals_ptr[it] = cacc::Vec3f(0.0f, 0.0f, 1.0f);
            data.values_ptr[it] = 0.0f;
            it += 1;
        }
        dvolume = cacc::PointCloud<cacc::DEVICE>::create<cacc::HOST>(hvolume);
    }

    {
        cudaStream_t stream;
        cudaStreamCreate(&stream);

        dim3 grid(cacc::divup(dcloud->cdata().num_vertices, KERNEL_BLOCK_SIZE));
        dim3 block(KERNEL_BLOCK_SIZE);

        for (std::size_t i = 0; i < overts.size(); ++i) {
            evaluate_position<<<grid, block, 0, stream>>>(i, args.max_distance,
                dbvh_tree->cdata(), dcloud->cdata(), dvolume->cdata());
        }

        cudaStreamDestroy(stream);
        CHECK(cudaDeviceSynchronize());
    }

    *hvolume = *dvolume;
    {
        cacc::PointCloud<cacc::HOST>::Data data = hvolume->cdata();
        for (std::size_t i = 0, it = 0; i < overts.size(); ++i) {
            if (ovalues[i] == -1.0f) continue;
            ovalues[i] = data.values_ptr[it++];
        }
    }

    mve::geom::SavePLYOptions opts;
    opts.write_vertex_values = true;
    mve::geom::save_ply_mesh(ocloud, args.ovolume, opts);

    return EXIT_SUCCESS;
}
