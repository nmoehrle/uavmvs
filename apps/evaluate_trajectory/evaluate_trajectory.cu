#include <chrono>
#include <atomic>
#include <iostream>

#include <cuda_runtime.h>

#include "util/arguments.h"
#include "util/file_system.h"

#include "util/io.h"

#include "mve/mesh_io_ply.h"
#include "mve/scene.h"

#include "acc/bvh_tree.h"

#include "cacc/point_cloud.h"
#include "cacc/util.h"
#include "cacc/math.h"
#include "cacc/matrix.h"
#include "cacc/reduction.h"

#include "col/mpl_viridis.h"

#include "eval/kernels.h"

#include "utp/trajectory.h"
#include "utp/trajectory_io.h"

typedef unsigned char uchar;

struct Arguments {
    std::string trajectory;
    std::string proxy_mesh;
    std::string proxy_cloud;
    std::string export_cloud;
    float max_distance;
};

Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_maxnum(3);
    args.set_nonopt_minnum(3);
    args.set_usage("Usage: " + std::string(argv[0]) + " [OPTS] TRAJECTORY/SCENE PROXY_MESH PROXY_CLOUD");
    args.add_option('e', "export", true, "export per surface point reconstructability as point cloud");
    args.add_option('\0', "max-distance", true, "maximum distance to surface [80.0]");
    args.set_description("Evaluate trajectory");
    args.parse(argc, argv);

    Arguments conf;
    conf.trajectory = args.get_nth_nonopt(0);
    conf.proxy_mesh = args.get_nth_nonopt(1);
    conf.proxy_cloud = args.get_nth_nonopt(2);
    conf.max_distance = 80.0f;

    for (util::ArgResult const* i = args.next_option();
         i != nullptr; i = args.next_option()) {
        switch (i->opt->sopt) {
        case 'e':
            conf.export_cloud = i->arg;
        break;
        case '\0':
            if (i->opt->lopt == "max-distance") {
                conf.max_distance = i->get_arg<float>();
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

int main(int argc, char * argv[])
{
    Arguments args = parse_args(argc, argv);

    cacc::select_cuda_device(3, 5);

    std::vector<mve::CameraInfo> trajectory;
    if (util::fs::dir_exists(args.trajectory.c_str())) {
        load_scene_as_trajectory(args.trajectory, &trajectory);
    } else if (util::fs::file_exists(args.trajectory.c_str())) {
        utp::load_trajectory(args.trajectory, &trajectory);
    } else {
        std::cerr << "Could not load trajectory" << std::endl;
        return EXIT_FAILURE;
    }

    cacc::BVHTree<cacc::DEVICE>::Ptr dbvh_tree;
    {
        acc::BVHTree<uint, math::Vec3f>::Ptr bvh_tree;
        bvh_tree = load_mesh_as_bvh_tree(args.proxy_mesh);
        dbvh_tree = cacc::BVHTree<cacc::DEVICE>::create<uint, math::Vec3f>(bvh_tree);
    }

    cacc::PointCloud<cacc::HOST>::Ptr cloud;
    cloud = load_point_cloud(args.proxy_cloud);
    cacc::PointCloud<cacc::DEVICE>::Ptr dcloud;
    dcloud = cacc::PointCloud<cacc::DEVICE>::create<cacc::HOST>(cloud);

    uint num_verts = dcloud->cdata().num_vertices;
    uint max_cameras = 20;

    cacc::VectorArray<cacc::Vec3f, cacc::DEVICE>::Ptr ddir_hist;
    ddir_hist = cacc::VectorArray<cacc::Vec3f, cacc::DEVICE>::create(num_verts, max_cameras);
    cacc::Array<float, cacc::DEVICE>::Ptr drecons;
    drecons = cacc::Array<float, cacc::DEVICE>::create(num_verts);
    drecons->null();

    std::cout << '\n';

    int width = 1920;
    int height = 1080;
    math::Matrix4f w2c;
    math::Matrix3f calib;
    math::Vec3f view_pos(0.0f);

    std::chrono::time_point<std::chrono::high_resolution_clock> start, end;

    std::cout << "Computing reconstuctability" << std::endl;
    start = std::chrono::high_resolution_clock::now();
    {
        cudaStream_t stream;
        cudaStreamCreate(&stream);
        dim3 grid(cacc::divup(num_verts, KERNEL_BLOCK_SIZE));
        dim3 block(KERNEL_BLOCK_SIZE);

        for (mve::CameraInfo const & cam : trajectory) {
            cam.fill_calibration(calib.begin(), width, height);
            cam.fill_world_to_cam(w2c.begin());
            cam.fill_camera_pos(view_pos.begin());

            update_direction_histogram<<<grid, block, 0, stream>>>(
                true, cacc::Vec3f(view_pos.begin()), args.max_distance,
                cacc::Mat4f(w2c.begin()), cacc::Mat3f(calib.begin()), width, height,
                dbvh_tree->accessor(), dcloud->cdata(), ddir_hist->cdata()
            );
        }

        cudaStreamDestroy(stream);
        CHECK(cudaDeviceSynchronize());
    }
    end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff = end - start;
    std::cout << "  GPU: " << diff.count() << 's' << std::endl;

    {
        dim3 grid(cacc::divup(num_verts, 2));
        dim3 block(32, 2);
        process_direction_histogram<<<grid, block>>>(
            ddir_hist->cdata());
    }

    {
        dim3 grid(cacc::divup(num_verts, KERNEL_BLOCK_SIZE));
        dim3 block(KERNEL_BLOCK_SIZE);
        evaluate_direction_histogram<<<grid, block>>>(ddir_hist->cdata(), drecons->cdata());
        CHECK(cudaDeviceSynchronize());
    }

    std::vector<float> values(num_verts);

    cacc::Array<float, cacc::HOST> recons(*drecons);
    cacc::Array<float, cacc::HOST>::Data const & data = recons.cdata();
    for (std::size_t i = 0; i < num_verts; ++i) {
        values[i] = data.data_ptr[i];
    }

    std::cout << "Average reconstructability" << std::endl;
    std::cout << "  GPU:\n"
        << "  " << cacc::reduction::sum(drecons) / num_verts << '\n'
        << "  " << cacc::reduction::min(drecons) << '\n'
        << "  " << cacc::reduction::max(drecons) << '\n'
        << std::endl;
    std::cout << "  CPU:\n"
        << "  " << std::accumulate(values.begin(), values.end(), 1.0f) / num_verts << '\n'
        << "  " << *std::min_element(values.begin(), values.end()) << '\n'
        << "  " << *std::max_element(values.begin(), values.end()) << '\n'
        << std::endl;

    std::cout << "Length: " << utp::length(trajectory) << '\n' << std::endl;

    if (!args.export_cloud.empty()) {
        mve::TriangleMesh::Ptr mesh;
        try {
            mesh = mve::geom::load_ply_mesh(args.proxy_cloud);
        } catch (std::exception& e) {
            std::cerr << "\tCould not load mesh: "<< e.what() << std::endl;
            std::exit(EXIT_FAILURE);
        }

        std::vector<float> & ovalues = mesh->get_vertex_values();
        ovalues.assign(values.begin(), values.end());

        mve::geom::SavePLYOptions opts;
        opts.write_vertex_values = true;
        mve::geom::save_ply_mesh(mesh, args.export_cloud, opts);
    }
}
