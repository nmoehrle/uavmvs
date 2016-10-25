#include <iostream>
#include <algorithm>
#include <unordered_set>

#include "util/system.h"
#include "util/arguments.h"
#include "util/file_system.h"

#include "mve/camera.h"
#include "mve/mesh_io_ply.h"

#include "math/bspline.h"

#include "cacc/util.h"
#include "cacc/math.h"
#include "cacc/tracing.h"
#include "cacc/nnsearch.h"
#include "cacc/reduction.h"

#include "util/io.h"

#include "geom/sphere.h"
#include "geom/volume_io.h"

#include "utp/trajectory.h"
#include "utp/trajectory_io.h"

#include "eval/kernels.h"

struct Arguments {
    std::string in_trajectory;
    std::string proxy_mesh;
    std::string proxy_cloud;
    std::string out_trajectory;
    uint max_iters;
    float min_distance;
    float max_distance;
    float focal_length;
};

Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_minnum(4);
    args.set_nonopt_maxnum(4);
    args.set_usage("Usage: " + std::string(argv[0]) + " [OPTS] IN_TRAJECTORY PROXY_MESH PROXY_CLOUD OUT_TRAJECTORY");
    args.set_description("Template app");
    args.add_option('\0', "min-distance", true, "minimum distance to surface [0.0]");
    args.add_option('\0', "max-distance", true, "maximum distance to surface [80.0]");
    args.add_option('\0', "focal-length", true, "camera focal length [0.86]");
    args.add_option('m', "max-iters", true, "maximum iterations [100]");
    args.parse(argc, argv);

    Arguments conf;
    conf.in_trajectory = args.get_nth_nonopt(0);
    conf.proxy_mesh = args.get_nth_nonopt(1);
    conf.proxy_cloud = args.get_nth_nonopt(2);
    conf.out_trajectory = args.get_nth_nonopt(3);
    conf.max_iters = 100;
    conf.min_distance = 0.0f;
    conf.max_distance = 80.0f;
    conf.focal_length = 0.86f;

    for (util::ArgResult const* i = args.next_option();
         i != 0; i = args.next_option()) {
        switch (i->opt->sopt) {
        case 'm':
            conf.max_iters = i->get_arg<uint>();
        break;
        case '\0':
            if (i->opt->lopt == "focal-length") {
                conf.focal_length = i->get_arg<float>();
            } else if (i->opt->lopt == "min-distance") {
                conf.min_distance = i->get_arg<float>();
            } else if (i->opt->lopt == "max-distance") {
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

float const pi = std::acos(-1.0f);

int main(int argc, char **argv) {
    util::system::register_segfault_handler();
    util::system::print_build_timestamp(argv[0]);

    Arguments args = parse_args(argc, argv);

    int device = cacc::select_cuda_device(3, 5);

    cacc::BVHTree<cacc::DEVICE>::Ptr dbvh_tree;
    {
        acc::BVHTree<uint, math::Vec3f>::Ptr bvh_tree;
        bvh_tree = load_mesh_as_bvh_tree(args.proxy_mesh);
        dbvh_tree = cacc::BVHTree<cacc::DEVICE>::create<uint, math::Vec3f>(bvh_tree);
    }
    cacc::tracing::bind_textures(dbvh_tree->cdata());

    uint num_sverts;
    cacc::KDTree<3u, cacc::DEVICE>::Ptr dkd_tree;
    {
        mve::TriangleMesh::Ptr sphere = generate_sphere(1.0f, 3u);
        std::vector<math::Vec3f> const & verts = sphere->get_vertices();
        num_sverts = verts.size();
        acc::KDTree<3u, uint>::Ptr kd_tree = acc::KDTree<3, uint>::create(verts);
        dkd_tree = cacc::KDTree<3u, cacc::DEVICE>::create<uint>(kd_tree);
    }
    cacc::nnsearch::bind_textures(dkd_tree->cdata());

    cacc::PointCloud<cacc::DEVICE>::Ptr dcloud;
    {
        cacc::PointCloud<cacc::HOST>::Ptr cloud;
        cloud = load_point_cloud(args.proxy_cloud);
        dcloud = cacc::PointCloud<cacc::DEVICE>::create<cacc::HOST>(cloud);
    }
    uint num_verts = dcloud->cdata().num_vertices;

    acc::KDTree<3, uint>::Ptr kd_tree(load_mesh_as_kd_tree(args.proxy_cloud));

    uint max_cameras = 20;

    cacc::VectorArray<cacc::Vec3f, cacc::DEVICE>::Ptr ddir_hist;
    ddir_hist = cacc::VectorArray<cacc::Vec3f, cacc::DEVICE>::create(num_verts, max_cameras);
    cacc::Array<float, cacc::DEVICE>::Ptr drecons;
    drecons = cacc::Array<float, cacc::DEVICE>::create(num_verts);

    mve::CameraInfo cam;
    cam.flen = args.focal_length;
    math::Matrix3f calib;
    int width = 1920;
    int height = 1080;
    cam.fill_calibration(calib.begin(), width, height);

    std::array<math::Vec3f, 27> offsets;
    for (int z = 0; z < 3; ++z) {
        for (int y = 0; y < 3; ++y) {
            for (int x = 0; x < 3; ++x) {
                int idx = (z * 3 + y) * 3 + x;
                offsets[idx][0] = (x - 1) * 1.0f;
                offsets[idx][1] = (y - 1) * 1.0f;
                offsets[idx][2] = (z - 1) * 1.0f;
            }
        }
    }
    float scale = args.max_distance / 25.0f;

    std::vector<mve::CameraInfo> trajectory;
    utp::load_trajectory(args.in_trajectory, &trajectory);

    std::mt19937 gen(12345);
    std::vector<std::size_t> indices(trajectory.size());
    std::iota(indices.begin(), indices.end(), 0);

    float min_sq_distance = args.max_distance * args.max_distance;

    std::vector<std::size_t> oindices;
    #pragma omp parallel
    {
        cacc::set_cuda_device(device);

        cudaStream_t stream;
        cudaStreamCreate(&stream);

        cacc::VectorArray<float, cacc::DEVICE>::Ptr dcon_hist;
        dcon_hist = cacc::VectorArray<float, cacc::DEVICE>::create(num_sverts, 1, stream);

        cacc::Image<float, cacc::DEVICE>::Ptr dhist;
        dhist = cacc::Image<float, cacc::DEVICE>::create(128, 45, stream);
        cacc::Image<float, cacc::HOST>::Ptr hist;
        hist = cacc::Image<float, cacc::HOST>::create(128, 45, stream);

        for (uint i = 0; i < args.max_iters; ++i) {
            #pragma omp single
            {
                std::shuffle(indices.begin(), indices.end(), gen);
                drecons->null();
                ddir_hist->clear();

                oindices.clear();
                {
                    std::vector<math::Vec3f> poss;
                    for (std::size_t idx : indices) {
                        math::Vec3f pos;
                        trajectory[idx].fill_camera_pos(pos.begin());

                        bool too_close = std::any_of(poss.begin(), poss.end(),
                            [&pos, &min_sq_distance](math::Vec3f const & opos) -> bool {
                                return (pos - opos).square_norm() < min_sq_distance;
                        });

                        if (!too_close) {
                            oindices.push_back(idx);
                            poss.push_back(pos);
                        }
                    }
                }

                std::unordered_set<std::size_t> idxset(oindices.begin(), oindices.end());
                for (std::size_t j = 0; j < trajectory.size(); ++j) {
                    if (idxset.count(j)) continue;
                    mve::CameraInfo const & cam = trajectory[j];

                    math::Vec3f pos;
                    cam.fill_camera_pos(pos.begin());
                    math::Matrix4f w2c;
                    cam.fill_world_to_cam(w2c.begin());

                    dim3 grid(cacc::divup(num_verts, KERNEL_BLOCK_SIZE));
                    dim3 block(KERNEL_BLOCK_SIZE);
                    populate_direction_histogram<<<grid, block, 0, stream>>>(
                        cacc::Vec3f(pos.begin()), args.max_distance,
                        cacc::Mat4f(w2c.begin()), cacc::Mat3f(calib.begin()), width, height,
                        dbvh_tree->cdata(), dcloud->cdata(), drecons->cdata(), ddir_hist->cdata()
                    );
                }

                cudaStreamSynchronize(stream);
            }

            #pragma omp for schedule(dynamic)
            for (std::size_t j = 0; j < oindices.size(); ++j) {
                std::size_t idx = oindices[j];
                mve::CameraInfo & cam = trajectory[idx];
                math::Vec3f cpos;
                cam.fill_camera_pos(cpos.begin());

                float vmax = 0.0f;

                for (int k = 0; k < 27; ++k) {
                    math::Vec3f pos = cpos + scale * offsets[k] * (1.0f - i / (args.max_iters / 2.0f));

                    #if 0
                    if (i > args.max_iters / 2) {
                        math::BSpline<math::Vec3f> spline;
                        for (std::size_t h = 0; h < 3; ++h) {
                            mve::CameraInfo & cam = trajectory[std::max(0ul, idx - h)];
                            math::Vec3f pos;
                            cam.fill_camera_pos(pos.begin());
                            spline.add_point(pos);
                        }
                        spline.add_point(pos);
                        for (std::size_t h = 0; h < 3; ++h) {
                            mve::CameraInfo & cam = trajectory[std::min(idx + h, trajectory.size() - 1)];
                            math::Vec3f pos;
                            cam.fill_camera_pos(pos.begin());
                            spline.add_point(pos);
                        }
                        spline.uniform_knots(0.0, 1.0f);

                        pos = spline.evaluate(0.5f);
                    }
                    #endif

                    if (kd_tree->find_nn(pos, nullptr, args.min_distance)) continue;

                    dcon_hist->null();
                    {
                        dim3 grid(cacc::divup(num_verts, KERNEL_BLOCK_SIZE));
                        dim3 block(KERNEL_BLOCK_SIZE);
                        populate_histogram<<<grid, block, 0, stream>>>(
                            cacc::Vec3f(pos.begin()),
                            args.max_distance, dbvh_tree->cdata(), dcloud->cdata(), dkd_tree->cdata(),
                            ddir_hist->cdata(), drecons->cdata(), dcon_hist->cdata());
                    }

                    {
                        dim3 grid(cacc::divup(128, KERNEL_BLOCK_SIZE), 45);
                        dim3 block(KERNEL_BLOCK_SIZE);
                        evaluate_histogram<<<grid, block, 0, stream>>>(cacc::Mat3f(calib.begin()), width, height,
                            dkd_tree->cdata(), dcon_hist->cdata(), dhist->cdata());
                    }

                    *hist = *dhist;
                    cacc::Image<float, cacc::HOST>::Data data = hist->cdata();

                    hist->sync();

                    //TODO write a kernel to select best viewing direction

                    float max = 0.0f;
                    float theta = 0.0f;
                    float phi = 0.0f;
                    for (int y = 0; y < data.height; ++y) {
                        for (int x = 0; x < data.width; ++x) {
                            float v = data.data_ptr[y * data.pitch / sizeof(float) + x];
                            if (v > max) {
                                max = v;
                                theta = (0.5f + (y / (float) data.height) / 2.0f) * pi;
                                phi = (x / (float) data.width) * 2.0f * pi;
                            }
                        }
                    }

                    if (max < vmax) continue;

                    vmax = max;

                    math::Matrix3f rot = utp::rotation_from_spherical(theta, phi);

                    math::Vec3f trans = -rot * pos;

                    std::copy(trans.begin(), trans.end(), cam.trans);
                    std::copy(rot.begin(), rot.end(), cam.rot);
                }
            }

            #pragma omp single
            {
                /* Compute current reconstructability (every nth iteration?) */
                for (std::size_t j = 0; j < oindices.size(); ++j) {
                    mve::CameraInfo & cam = trajectory[oindices[j]];
                    math::Vec3f pos;
                    cam.fill_camera_pos(pos.begin());
                    math::Matrix4f w2c;
                    cam.fill_world_to_cam(w2c.begin());
                    {
                        dim3 grid(cacc::divup(num_verts, KERNEL_BLOCK_SIZE));
                        dim3 block(KERNEL_BLOCK_SIZE);
                        populate_direction_histogram<<<grid, block, 0, stream>>>(
                            cacc::Vec3f(pos.begin()), args.max_distance,
                            cacc::Mat4f(w2c.begin()), cacc::Mat3f(calib.begin()), width, height,
                            dbvh_tree->cdata(), dcloud->cdata(), drecons->cdata(), ddir_hist->cdata()
                        );
                    }
                }

                float length = utp::length(trajectory);

                float avg_recon = cacc::sum(drecons) / num_verts;
                std::cout << i << "(" << oindices.size() << ") " << avg_recon << " " << length << std::endl;
            }
        }
        cudaStreamDestroy(stream);
    }

    utp::save_trajectory(trajectory, args.out_trajectory);

    return EXIT_SUCCESS;
}
