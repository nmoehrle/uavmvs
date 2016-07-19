#include <fstream>
#include <iostream>

#include "fmt/format.h"

#include "util/system.h"
#include "util/arguments.h"
#include "util/choices.h"

#include "mve/camera.h"
#include "mve/mesh_io_ply.h"
#include "mve/image_io.h"

#include "cacc/util.h"
#include "cacc/math.h"
#include "cacc/tracing.h"
#include "cacc/nnsearch.h"

#include "eval/kernels.h"

struct Arguments {
    std::string proxy_mesh;
    std::string proxy_cloud;
    std::string proxy_sphere;
    std::string trajectory;
    float min_distance;
    float max_velocity;
};

Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_minnum(4);
    args.set_nonopt_maxnum(4);
    args.set_usage("Usage: " + std::string(argv[0])
        + " [OPTS] PROXY_MESH PROXY_CLOUD PROXY_SPHERE TRAJECTORY");
    args.set_description("Plans a trajectory maximizing reconstructability");
    args.parse(argc, argv);

    Arguments conf;
    conf.proxy_mesh = args.get_nth_nonopt(0);
    conf.proxy_cloud = args.get_nth_nonopt(1);
    conf.proxy_sphere = args.get_nth_nonopt(2);
    conf.trajectory = args.get_nth_nonopt(3);

    for (util::ArgResult const* i = args.next_option();
         i != 0; i = args.next_option()) {
        switch (i->opt->sopt) {
        default:
            throw std::invalid_argument("Invalid option");
        }
    }

    return conf;
}

void load_trajectory(std::string const & path,
        std::vector<mve::CameraInfo> * trajectory)
{
    std::ifstream in(path.c_str());
    if (!in.good()) throw std::runtime_error("Could not open trajectory file");
    std::size_t length;
    in >> length;

    trajectory->resize(length);

    for (std::size_t i = 0; i < length; ++i) {
        math::Vec3f pos;
        for (int j = 0; j < 3; ++j) {
            in >> pos[j];
        }
        math::Matrix3f rot;
        for (int j = 0; j < 9; ++j) {
            in >> rot[j];
        }
        math::Vec3f trans = -rot * pos;

        mve::CameraInfo & cam = trajectory->at(i);
        cam.flen = 0.86f; //TODO save and read from file
        std::copy(trans.begin(), trans.end(), cam.trans);
        std::copy(rot.begin(), rot.end(), cam.rot);
    }

    if (in.fail()) {
        in.close();
        throw std::runtime_error("Invalid trajectory file");
    }

    in.close();
}


acc::KDTree<3, uint>::Ptr
load_mesh_as_kd_tree(std::string const & path)
{
    mve::TriangleMesh::Ptr mesh;
    try {
        mesh = mve::geom::load_ply_mesh(path);
    } catch (std::exception& e) {
        std::cerr << "\tCould not load mesh: "<< e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }

    std::vector<math::Vec3f> const & vertices = mesh->get_vertices();
    return acc::KDTree<3, uint>::create(vertices);
}

acc::BVHTree<uint, math::Vec3f>::Ptr
load_mesh_as_bvh_tree(std::string const & path)
{
    mve::TriangleMesh::Ptr mesh;
    try {
        mesh = mve::geom::load_ply_mesh(path);
    } catch (std::exception& e) {
        std::cerr << "\tCould not load mesh: "<< e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }

    std::vector<math::Vec3f> const & vertices = mesh->get_vertices();
    std::vector<uint> const & faces = mesh->get_faces();
    return acc::BVHTree<uint, math::Vec3f>::create(faces, vertices);
}

cacc::PointCloud<cacc::HOST>::Ptr
load_point_cloud(std::string const & path)
{
    mve::TriangleMesh::Ptr mesh;
    try {
        mesh = mve::geom::load_ply_mesh(path);
    } catch (std::exception& e) {
        std::cerr << "\tCould not load mesh: " << e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }
    mesh->ensure_normals(true, true);

    std::vector<math::Vec3f> const & vertices = mesh->get_vertices();
    std::vector<math::Vec3f> const & normals = mesh->get_vertex_normals();

    cacc::PointCloud<cacc::HOST>::Ptr ret;
    ret = cacc::PointCloud<cacc::HOST>::create(vertices.size());
    cacc::PointCloud<cacc::HOST>::Data data = ret->cdata();
    for (std::size_t i = 0; i < vertices.size(); ++i) {
        data.vertices_ptr[i] = cacc::Vec3f(vertices[i].begin());
        data.normals_ptr[i] = cacc::Vec3f(normals[i].begin());
    }

    return ret;
}

int main(int argc, char **argv) {
    util::system::register_segfault_handler();
    util::system::print_build_timestamp(argv[0]);

    Arguments args = parse_args(argc, argv);

    cacc::select_cuda_device(3, 5);

    mve::TriangleMesh::Ptr mesh;
    try {
        mesh = mve::geom::load_ply_mesh(args.proxy_sphere);
    } catch (std::exception& e) {
        std::cerr << "\tCould not load mesh: "<< e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }

    std::vector<math::Vec3f> & overtices = mesh->get_vertices();
    std::vector<math::Vec3f> vertices = mesh->get_vertices();
    std::vector<float> & ovalues = mesh->get_vertex_values();
    ovalues.resize(vertices.size());

    acc::KDTree<3u, uint>::Ptr kd_tree;
    kd_tree = load_mesh_as_kd_tree(args.proxy_sphere);
    cacc::KDTree<3u, cacc::DEVICE>::Ptr dkd_tree;
    dkd_tree = cacc::KDTree<3u, cacc::DEVICE>::create<uint>(kd_tree);
    cacc::nnsearch::bind_textures(dkd_tree->cdata());

    acc::BVHTree<uint, math::Vec3f>::Ptr bvh_tree;
    bvh_tree = load_mesh_as_bvh_tree(args.proxy_mesh);
    cacc::BVHTree<cacc::DEVICE>::Ptr dbvh_tree;
    dbvh_tree = cacc::BVHTree<cacc::DEVICE>::create<uint, math::Vec3f>(bvh_tree);
    cacc::tracing::bind_textures(dbvh_tree->cdata());

    cacc::PointCloud<cacc::HOST>::Ptr cloud;
    cloud = load_point_cloud(args.proxy_cloud);
    cacc::PointCloud<cacc::DEVICE>::Ptr dcloud;
    dcloud = cacc::PointCloud<cacc::DEVICE>::create<cacc::HOST>(cloud);

    uint num_vertices = dcloud->cdata().num_vertices;
    uint max_cameras = 20;

    cacc::VectorArray<cacc::Vec2f, cacc::DEVICE>::Ptr ddir_hist;
    ddir_hist = cacc::VectorArray<cacc::Vec2f, cacc::DEVICE>::create(num_vertices, max_cameras);
    cacc::VectorArray<float, cacc::HOST>::Ptr con_hist;
    con_hist = cacc::VectorArray<float, cacc::HOST>::create(ovalues.size(), 2);
    cacc::VectorArray<float, cacc::HOST>::Data data = con_hist->cdata();
    cacc::VectorArray<float, cacc::DEVICE>::Ptr dcon_hist;
    dcon_hist = cacc::VectorArray<float, cacc::DEVICE>::create(ovalues.size(), 2);

    cacc::Image<float, cacc::DEVICE>::Ptr dhist;
    dhist = cacc::Image<float, cacc::DEVICE>::create(360, 180);

    math::Vec3f pos;
    math::Matrix3f calib;
    int width = 1920;
    int height = 1080;

    int cnt = 0;

    std::vector<mve::CameraInfo> trajectory;
    load_trajectory(args.trajectory, &trajectory);

    for (mve::CameraInfo const & cam : trajectory) {
        cam.fill_calibration(calib.begin(), width, height);
        cam.fill_camera_pos(pos.begin());

        //TODO write clear kernel
        for (std::size_t i = 0; i < ovalues.size(); ++i) {
            data.data_ptr[i] = 0.0f;
            data.data_ptr[i + data.pitch / sizeof(float)] = cacc::float_to_uint32(0.0f);
        }
        *dcon_hist = *con_hist;

        {
            dim3 grid(cacc::divup(num_vertices, KERNEL_BLOCK_SIZE));
            dim3 block(KERNEL_BLOCK_SIZE);
            populate_histogram<<<grid, block>>>(cacc::Vec3f(pos.begin()),
                dbvh_tree->cdata(), dcloud->cdata(), dkd_tree->cdata(),
                ddir_hist->cdata(), dcon_hist->cdata());
            CHECK(cudaDeviceSynchronize());
        }

        {
            dim3 grid(cacc::divup(360, KERNEL_BLOCK_SIZE), 180);
            dim3 block(KERNEL_BLOCK_SIZE);
            evaluate_histogram<<<grid, block>>>(cacc::Mat3f(calib.begin()), width, height,
                dkd_tree->cdata(), dcon_hist->cdata(), dhist->cdata());
            CHECK(cudaDeviceSynchronize());
        }

        {
            dim3 grid(cacc::divup(360, KERNEL_BLOCK_SIZE), 180);
            dim3 block(KERNEL_BLOCK_SIZE);
            evaluate_histogram<<<grid, block>>>(dkd_tree->cdata(),
               dhist->cdata(), dcon_hist->cdata());
            CHECK(cudaDeviceSynchronize());
        }

        *con_hist = *dcon_hist;
        for (std::size_t i = 0; i < vertices.size(); ++i) {
            overtices[i] = vertices[i] + pos;
#if 1
            float * f = data.data_ptr + data.pitch / sizeof(float) + i;
            uint32_t v = reinterpret_cast<uint32_t&>(*f);
            ovalues[i] = cacc::uint32_to_float(v);
#else
            ovalues[i] = data.data_ptr[i];
#endif
        }

        mve::geom::SavePLYOptions opts;
        opts.write_vertex_values = true;
        std::string filename = fmt::format("/tmp/test-sphere-{:04d}.ply", cnt++);
        mve::geom::save_ply_mesh(mesh, filename, opts);
    }

    return EXIT_SUCCESS;
}
