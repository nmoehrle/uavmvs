#include <chrono>
#include <atomic>
#include <iostream>

#include "util/arguments.h"
#include "mve/mesh_io_ply.h"
#include "mve/camera.h"
#include "acc/bvh_tree.h"
#include "cacc/point_cloud.h"
#include "cacc/util.h"
#include "cacc/matrix.h"
#include "cacc/bvh_tree.h"
#include "cacc/tracing.h"

#include "kernel.h"

inline
uint divup(uint a, uint b) {
    return a / b  + (a % b != 0);
}

cacc::PointCloud<cacc::HOST>::Ptr
load_point_cloud(std::string const &path)
{
    mve::TriangleMesh::Ptr mesh;
    try {
        mesh = mve::geom::load_ply_mesh(path);
    } catch (std::exception& e) {
        std::cerr << "\tCould not load mesh: "<< e.what() << std::endl;
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

void cpu(math::Matrix4f w2c, math::Matrix3f calib, math::Vec3f view_pos, int width, int height,
    acc::BVHTree<uint, math::Vec3f>::Ptr bvh_tree, cacc::PointCloud<cacc::HOST>::Ptr cloud)
{
    std::chrono::time_point<std::chrono::high_resolution_clock> start, end;
    std::atomic<uint> an(0);
    #pragma omp parallel
    {
        uint ln = 0;
        start = std::chrono::high_resolution_clock::now();
        #pragma omp for
        for (std::size_t j = 0; j < cloud->cdata().num_vertices; ++j) {
            math::Vec3f v; //TODO fix this mess
            for (int i = 0; i < 3; ++i) v[i] = cloud->cdata().vertices_ptr[j][i];

            math::Vec3f v2c = view_pos - v;
            float n = v2c.norm();
            if (n > 45.0f) continue;
            math::Vec3f pt = calib * w2c.mult(v, 1.0f);
            math::Vec2f p(pt[0] / pt[2] - 0.5f, pt[1] / pt[2] - 0.5f);

            if (p[0] < 0.0f || width <= p[0] || p[1] < 0.0f || height <= p[1]) continue;

            acc::Ray<math::Vec3f> ray;
            ray.origin = v + v2c * 0.01f;
            ray.dir = v2c / n;
            ray.tmin = 0.0f;
            ray.tmax = inf;

            if (bvh_tree->intersect(ray)) continue;

            ln +=1;
        }
        an += ln;
    }
    end = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> diff = end - start;
    std::cout << an << " " << diff.count() << std::endl;
}

void gpu(math::Matrix4f w2c, math::Matrix3f calib, math::Vec3f view_pos, int width, int height,
    cacc::BVHTree<cacc::DEVICE>::Ptr dbvh_tree, cacc::PointCloud<cacc::DEVICE>::Ptr dcloud)
{
    std::chrono::time_point<std::chrono::high_resolution_clock> start, end;
    uint n = 0;

    uint * n_ptr;
    cudaMalloc(&n_ptr, sizeof(uint));
    cudaMemset(n_ptr, 0, sizeof(uint));

    dim3 grid(divup(dcloud->cdata().num_vertices, KERNEL_BLOCK_SIZE));
    dim3 block(KERNEL_BLOCK_SIZE);

    start = std::chrono::high_resolution_clock::now();
    kernel<<<grid, block>>>(cacc::Mat4f(w2c.begin()), cacc::Mat3f(calib.begin()),
        cacc::Vec3f(view_pos.begin()), width, height,
        dbvh_tree->cdata(), dcloud->cdata(), n_ptr);
    cudaDeviceSynchronize();
    end = std::chrono::high_resolution_clock::now();

    cudaMemcpy(&n, n_ptr, sizeof(uint), cudaMemcpyDeviceToHost);
    cudaFree(n_ptr);

    std::chrono::duration<double> diff = end - start;
    std::cout << n << " " << diff.count() << std::endl;
}

void load_trajectory(std::vector<mve::CameraInfo> * trajectory) {
    mve::CameraInfo cam;
    cam.flen = 1.0f;
    cam.trans[0] = -10.0f;
    cam.trans[1] = -10.0f;
    cam.trans[2] = 20.0f;
    cam.rot[8] = -1;
    trajectory->push_back(cam);
}

struct Arguments {
    std::string proxy_mesh;
    std::string proxy_cloud;
};

Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_maxnum(2);
    args.set_nonopt_minnum(2);
    args.set_usage("Usage: " + std::string(argv[0]) + " [OPTS] PROXY_MESH PROXY_CLOUD");
    args.set_description("Evaluate trajectory");
    args.parse(argc, argv);

    Arguments conf;
    conf.proxy_mesh = args.get_nth_nonopt(0);
    conf.proxy_cloud = args.get_nth_nonopt(1);

    for (util::ArgResult const* i = args.next_option();
         i != nullptr; i = args.next_option()) {
        switch (i->opt->sopt) {
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

    acc::BVHTree<uint, math::Vec3f>::Ptr bvh_tree;
    bvh_tree = load_mesh_as_bvh_tree(args.proxy_mesh);
    cacc::BVHTree<cacc::DEVICE>::Ptr dbvh_tree;
    dbvh_tree = cacc::BVHTree<cacc::DEVICE>::create<uint, math::Vec3f>(bvh_tree);
    cacc::tracing::load_textures(dbvh_tree->cdata());

    cacc::PointCloud<cacc::HOST>::Ptr cloud;
    cloud = load_point_cloud(args.proxy_cloud);
    cacc::PointCloud<cacc::DEVICE>::Ptr dcloud;
    dcloud = cacc::PointCloud<cacc::DEVICE>::create<cacc::HOST>(cloud);

    std::vector<mve::CameraInfo> trajectory;
    load_trajectory(&trajectory);

    //TESTING - one view ...

    mve::CameraInfo cam = trajectory.front();

    int width = 1920;
    int height = 1080;
    math::Matrix4f w2c;
    math::Matrix3f calib;
    math::Vec3f view_pos(0.0f);
    cam.fill_calibration(calib.begin(), width, height);
    cam.fill_cam_to_world(w2c.begin());
    cam.fill_camera_pos(view_pos.begin());

    cpu(w2c, calib, view_pos, width, height, bvh_tree, cloud);
    gpu(w2c, calib, view_pos, width, height, dbvh_tree, dcloud);
}
