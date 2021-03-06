/*
 * Copyright (C) 2016-2018, Nils Moehrle
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include <random>
#include <csignal>
#include <iostream>
#include <algorithm>

#include "util/system.h"
#include "util/arguments.h"
#include "util/file_system.h"

#include "mve/camera.h"
#include "mve/image_io.h"
#include "mve/mesh_io_ply.h"

#include "math/bspline.h"
#include "math/geometry.h"

#include "cacc/util.h"
#include "cacc/math.h"
#include "cacc/nnsearch.h"
#include "cacc/reduction.h"

#include "util/io.h"
#include "util/cio.h"

#include "geom/sphere.h"
#include "geom/volume_io.h"

#include "utp/trajectory.h"
#include "utp/trajectory_io.h"

#include "eval/kernels.h"

#include "opti/nelder_mead.h"

struct Arguments {
    std::string in_trajectory;
    std::string proxy_mesh;
    std::string proxy_cloud;
    std::string airspace;
    std::string out_trajectory;
    std::uint32_t seed;
    uint max_iters;
    float min_distance;
    float max_distance;
    float focal_length;
    float target_recon;
    float independence;
};

Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_minnum(5);
    args.set_nonopt_maxnum(5);
    args.set_usage("Usage: " + std::string(argv[0])
        + " [OPTS] IN_TRAJECTORY PROXY_MESH PROXY_CLOUD AIRSPACE OUT_TRAJECTORY");
    args.set_description("Optimize position and orientation of trajectory views.");
    args.add_option('\0', "seed", true, "seed for RNG [0]");
    args.add_option('\0', "min-distance", true, "minimum distance to surface [2.5]");
    args.add_option('\0', "max-distance", true, "maximum distance to surface [50.0]");
    args.add_option('\0', "focal-length", true, "camera focal length [0.86]");
    args.add_option('\0', "independence", true, "reduce independence constraint [1.0]");
    args.add_option('m', "max-iters", true, "maximum iterations [100]");
    args.parse(argc, argv);

    Arguments conf;
    conf.in_trajectory = args.get_nth_nonopt(0);
    conf.proxy_mesh = args.get_nth_nonopt(1);
    conf.proxy_cloud = args.get_nth_nonopt(2);
    conf.airspace = args.get_nth_nonopt(3);
    conf.out_trajectory = args.get_nth_nonopt(4);
    conf.seed = 0u;
    conf.max_iters = 100;
    conf.min_distance = 2.5f;
    conf.max_distance = 50.0f;
    conf.focal_length = 0.86f;
    conf.target_recon = 3.0f;
    conf.independence = 1.0f;

    for (util::ArgResult const* i = args.next_option();
         i != 0; i = args.next_option()) {
        switch (i->opt->sopt) {
        case 'm':
            conf.max_iters = i->get_arg<uint>();
        break;
        case '\0':
            if (i->opt->lopt == "seed") {
                conf.seed = i->get_arg<std::uint32_t>();
            } else if (i->opt->lopt == "focal-length") {
                conf.focal_length = i->get_arg<float>();
            } else if (i->opt->lopt == "min-distance") {
                conf.min_distance = i->get_arg<float>();
            } else if (i->opt->lopt == "max-distance") {
                conf.max_distance = i->get_arg<float>();
            } else if (i->opt->lopt == "independence") {
                conf.independence = i->get_arg<float>();
            } else {
                throw std::invalid_argument("Invalid option");
            }
        break;
        default:
            throw std::invalid_argument("Invalid option");
        }
    }

    assert(conf.min_distance >= 0.0f);

    return conf;
}

float const pi = std::acos(-1.0f);

bool terminate = false;

void initialize(Simplex<3> * simplex, math::Vec3f pos, float scale, float theta, float phi) {
    static float tmp = 1.0f / std::sqrt(2.0f);
    static math::Vec3f tet[] = {
        {1.0f , 0.0f, - tmp},
        {-1.0f , 0.0f, - tmp},
        {0.0f, 1.0f, tmp},
        {0.0f, -1.0f, tmp}
    };

    math::Matrix3f rot = utp::rotation_from_spherical(theta, phi);

    std::array<math::Vector<float, 3>, 4> & v = simplex->verts;
    for (std::size_t j = 0; j < v.size(); ++j) {
        v[j] = pos + rot * (tet[j] * scale);
    }
}

int main(int argc, char **argv) {
    util::system::register_segfault_handler();
    util::system::print_build_timestamp(argv[0]);

    Arguments args = parse_args(argc, argv);

    int device = cacc::select_cuda_device(3, 5);

    /* Load proxy mesh and construct BVH for visibility calculations. */
    cacc::BVHTree<cacc::DEVICE>::Ptr dbvh_tree;
    {
        acc::BVHTree<uint, math::Vec3f>::Ptr bvh_tree;
        bvh_tree = load_mesh_as_bvh_tree(args.proxy_mesh);
        dbvh_tree = cacc::BVHTree<cacc::DEVICE>::create<uint, math::Vec3f>(bvh_tree);
    }

    /* Generate sphere and construct KDTree for histogram binning. */
    uint num_sverts;
    cacc::KDTree<3u, cacc::DEVICE>::Ptr dkd_tree;
    {
        mve::TriangleMesh::Ptr sphere = generate_sphere_mesh(1.0f, 3u);
        std::vector<math::Vec3f> const & verts = sphere->get_vertices();
        num_sverts = verts.size();
        acc::KDTree<3u, uint>::Ptr kd_tree = acc::KDTree<3, uint>::create(verts);
        dkd_tree = cacc::KDTree<3u, cacc::DEVICE>::create<uint>(kd_tree);
    }

    /* Load proxy cloud to evaluate heuristic */
    cacc::PointCloud<cacc::DEVICE>::Ptr dcloud;
    {
        cacc::PointCloud<cacc::HOST>::Ptr cloud;
        cloud = load_point_cloud(args.proxy_cloud);
        dcloud = cacc::PointCloud<cacc::DEVICE>::create<cacc::HOST>(cloud);
    }
    int num_verts = dcloud->cdata().num_vertices;

    acc::BVHTree<uint, math::Vec3f>::Ptr bvh_tree;
    bvh_tree = load_mesh_as_bvh_tree(args.airspace);

    /* Allocate shared GPU data structures. */
    uint max_cameras = 32;
    cacc::VectorArray<cacc::Vec3f, cacc::DEVICE>::Ptr dobs_rays;
    dobs_rays = cacc::VectorArray<cacc::Vec3f, cacc::DEVICE>::create(num_verts, max_cameras);
    cacc::Array<float, cacc::DEVICE>::Ptr drecons;
    drecons = cacc::Array<float, cacc::DEVICE>::create(num_verts);
    cacc::Array<float, cacc::DEVICE>::Ptr dwrecons;
    dwrecons = cacc::Array<float, cacc::DEVICE>::create(num_verts);

    /* Reduce fov by 2 deg to compensate for inaccuracies. */
    float fov = 2.0f * std::atan2(1.0f, 2.0f * args.focal_length);
    fov -= 2.0f * (pi / 180.0f);

    mve::CameraInfo cam;
    cam.flen = 1.0f / (2.0f * std::tan(fov / 2.0f));
    math::Matrix3f calib;
    int width = 1920;
    int height = 1080;
    cam.fill_calibration(calib.begin(), width, height);

    std::vector<mve::CameraInfo> trajectory;
    utp::load_trajectory(args.in_trajectory, &trajectory);

    /* Initialize data structure for view selection distribution. */
    std::vector<std::size_t> iters(trajectory.size(), args.max_iters);
    std::mt19937 gen(args.seed);

    /* Determine minimal distance for independent views (ignoring simplex). */
    float min_sq_distance = args.independence * 2.0f
        * args.max_distance * args.max_distance;

    /* Initialize simplexes as regular tetrahedron. */
    std::vector<Simplex<3> > simplices(trajectory.size());
    {
        float scale = 0.5f * (args.min_distance / 10.0f);

        /* Randomly rotate simplexes of each camera. */
        std::uniform_real_distribution<float> dist(0.0f, pi);
        for (std::size_t i = 0; i < trajectory.size(); ++i) {
            mve::CameraInfo const & cam = trajectory[i];
            math::Vec3f pos;
            cam.fill_camera_pos(pos.begin());

            float theta = dist(gen);
            float phi = 2.0f * dist(gen);

            initialize(&simplices[i], pos, scale, theta, phi);
        }
    }

    std::signal(SIGINT, [] (int) -> void { terminate = true; });


    float volume = 0.0f;
    std::vector<std::size_t> oindices;
    std::vector<float> ovalues;
    ovalues.reserve(args.max_iters);
    #pragma omp parallel
    {
        /* Allocate thread local data structures. */
        cacc::set_cuda_device(device);

        cudaStream_t stream;
        CHECK(cudaStreamCreate(&stream));

        cudaEvent_t event;
        CHECK(cudaEventCreateWithFlags(&event, cudaEventDefault | cudaEventDisableTiming));

        /* Spherical histogram. */
        cacc::Array<float, cacc::DEVICE>::Ptr dcon_hist;
        dcon_hist = cacc::Array<float, cacc::DEVICE>::create(num_sverts, stream);

        /* Convoluted spherical histograms. */
        cacc::Image<float, cacc::DEVICE>::Ptr dhist;
        dhist = cacc::Image<float, cacc::DEVICE>::create(128, 45, stream);
        cacc::Image<float, cacc::HOST>::Ptr hist;
        hist = cacc::Image<float, cacc::HOST>::create(128, 45, stream);

        /* Initialize direction histograms. */
        #pragma omp for schedule(dynamic)
        for (std::size_t j = 0; j < trajectory.size(); ++j) {
            mve::CameraInfo const & cam = trajectory[j];

            math::Vec3f pos;
            cam.fill_camera_pos(pos.begin());
            math::Matrix4f w2c;
            cam.fill_world_to_cam(w2c.begin());

            dim3 grid(cacc::divup(num_verts, KERNEL_BLOCK_SIZE));
            dim3 block(KERNEL_BLOCK_SIZE);
            update_observation_rays<<<grid, block, 0, stream>>>(
                true, cacc::Vec3f(pos.begin()), args.max_distance,
                cacc::Mat4f(w2c.begin()), cacc::Mat3f(calib.begin()),
                width, height,
                dbvh_tree->accessor(),
                dcloud->cdata(), dobs_rays->cdata()
            );

            cacc::sync(stream, event, std::chrono::microseconds(100));
        }

        for (uint i = 0; i < args.max_iters && !terminate; ++i) {
            /* Select views to optimize by a single thread. */
            #pragma omp single
            {
                volume = 0.0f;
                oindices.clear();

                /* Create view selection distribution. */
                std::discrete_distribution<> d(iters.begin(), iters.end());
                {
                    /* Select multiple independent views for optimization. */
                    std::vector<math::Vec3f> poss;
                    for (std::size_t j = 0; j < trajectory.size(); ++j) {
                        std::size_t idx = d(gen);

                        math::Vec3f pos;
                        trajectory[idx].fill_camera_pos(pos.begin());

                        bool too_close = std::any_of(poss.begin(), poss.end(),
                            [&pos, &min_sq_distance](math::Vec3f const & opos) -> bool {
                                return (pos - opos).square_norm() < min_sq_distance;
                        });

                        if (!too_close) {
                            oindices.push_back(idx);
                            poss.push_back(pos);
                            iters[idx] -= 1;
                        }
                    }
                }
            }

            /* Remove view directions of selected views. */
            #pragma omp for schedule(dynamic)
            for (std::size_t j = 0; j < oindices.size(); ++j) {
                mve::CameraInfo const & cam = trajectory[oindices[j]];

                math::Vec3f pos;
                cam.fill_camera_pos(pos.begin());
                math::Matrix4f w2c;
                cam.fill_world_to_cam(w2c.begin());

                dim3 grid(cacc::divup(num_verts, KERNEL_BLOCK_SIZE));
                dim3 block(KERNEL_BLOCK_SIZE);
                update_observation_rays<<<grid, block, 0, stream>>>(
                    false, cacc::Vec3f(pos.begin()), args.max_distance,
                    cacc::Mat4f(w2c.begin()), cacc::Mat3f(calib.begin()),
                    width, height,
                    dbvh_tree->accessor(),
                    dcloud->cdata(), dobs_rays->cdata()
                );

                cacc::sync(stream, event, std::chrono::microseconds(100));
            }
            ((void)0); //For unknown reasons a statement is required here

            /* Compute new reconstructabilities. */
            #pragma omp single
            {
                {
                    dim3 grid(cacc::divup(num_verts, 2));
                    dim3 block(32, 2);
                    process_observation_rays<<<grid, block, 0, stream>>>(
                        dobs_rays->cdata());
                }

                {
                    dim3 grid(cacc::divup(num_verts, KERNEL_BLOCK_SIZE));
                    dim3 block(KERNEL_BLOCK_SIZE);
                    evaluate_observation_rays<<<grid, block, 0, stream>>>(
                        dobs_rays->cdata(), drecons->cdata());
                }

                cacc::sync(stream, event, std::chrono::microseconds(100));
            }

            /* Execute a iteration of simplex-downhill for selected views. */
            #pragma omp for schedule(dynamic)
            for (std::size_t j = 0; j < oindices.size(); ++j) {
                std::size_t idx = oindices[j];
                mve::CameraInfo & cam = trajectory[idx];

                /* Will be set by simplex-downhill. */
                float vmin = 0.0f;
                float vtheta = 0.0f;
                float vphi = 0.0f;

                /* Objective function for simplex-downhill. */
                std::function<float(math::Vec3f)> func =
                    [&] (math::Vec3f const & pos) -> float
                {
                    float w = 1.0f;

                    /* Return positive value if to close ground or surface. */
                    if (pos[2] < args.min_distance) return args.min_distance - pos[2];
                    std::pair<uint, math::Vec3f> cp;
                    if (bvh_tree->closest_point(pos, &cp, args.min_distance)) {
                        acc::Tri<math::Vec3f> const & tri = bvh_tree->get_triangle(cp.first);
                        math::Vec3f normal = calculate_normal(tri);
                        math::Vec3f cpp = pos - cp.second;
                        float dist = cpp.norm();
                        if (normal.dot(cpp) < 0.0f) {
                            return dist;
                        } else {
                            w = 1.0f - (args.min_distance - dist) / args.min_distance;
                        }
                    }

                    /* Clear spherical histogram. */
                    dcon_hist->null();
                    /* Compute spherical histogram. */
                    {
                        dim3 grid(cacc::divup(num_verts, KERNEL_BLOCK_SIZE));
                        dim3 block(KERNEL_BLOCK_SIZE);
                        populate_spherical_histogram<<<grid, block, 0, stream>>>(
                            cacc::Vec3f(pos.begin()), args.max_distance, args.target_recon,
                            dbvh_tree->accessor(), dcloud->cdata(), dkd_tree->accessor(),
                            dobs_rays->cdata(), drecons->cdata(), dcon_hist->cdata());
                    }

                    /* Convolve spherical histogram. */
                    {
                        dim3 grid(cacc::divup(128, KERNEL_BLOCK_SIZE), 45);
                        dim3 block(KERNEL_BLOCK_SIZE);
                        evaluate_spherical_histogram<<<grid, block, 0, stream>>>(
                            cacc::Mat3f(calib.begin()), width, height,
                            dkd_tree->accessor(), dcon_hist->cdata(), dhist->cdata());
                    }

                    *hist = *dhist;
                    cacc::Image<float, cacc::HOST>::Data data = hist->cdata();

                    cacc::sync(stream, event, std::chrono::microseconds(100));

                    /* Select optimal direction based on spherical histogram. */
                    float min = 0.0f; //all values are negative;
                    float theta = 0.0f;
                    float phi = 0.0f;

                    for (int y = 0; y < data.height; ++y) {
                        for (int x = 0; x < data.width; ++x) {
                            float v = w * data.data_ptr[y * data.pitch / sizeof(float) + x];
                            if (v < min) {
                                min = v;
                                theta = (0.5f + (y / (float) data.height) / 2.0f) * pi;
                                phi = (x / (float) data.width) * 2.0f * pi;
                            }
                        }
                    }

                    if (min < vmin) {
                        vmin = min;
                        vtheta = theta;
                        vphi = phi;
                    }

                    return min;
                };

                Simplex<3> & simplex  = simplices[idx];

                /* Execute one iteration of simplex-downhill and update view accordingly. */
                float value;
                std::size_t vid;
                std::tie(vid, value) = nelder_mead(&simplex, func);

                /* View does not contribute to the reconstruction, reinitialize. */
                if (value >= 0.0f) {
                    math::Vec3f trans(cam.trans);
                    math::Matrix3f rot(cam.rot);
                    math::Vec3f pos = -rot.transpose() * trans;

                    /* Move towards center and lift till we are in the flyable airspace. */
                    math::Vec3f center = math::Vec3f(0.0f, 0.0f, args.max_distance * 0.9f);
                    math::Vec3f dir = (center - pos).normalize();
                    pos += dir * args.min_distance;
                    while (bvh_tree->closest_point(pos, nullptr, args.min_distance)) {
                        pos[2] += args.min_distance;
                    }

                    float scale = 0.5f * (args.min_distance / 10.0f);
                    initialize(&simplex, pos, scale, 0.0f, 0.0f);
                }
                math::Vec3f pos = simplex.verts[vid];

                math::Matrix3f rot = utp::rotation_from_spherical(vtheta, vphi);
                math::Vec3f trans = -rot * pos;

                std::copy(trans.begin(), trans.end(), cam.trans);
                std::copy(rot.begin(), rot.end(), cam.rot);
            }

            /* Add view directions of optimized views. */
            #pragma omp for schedule(dynamic)
            for (std::size_t j = 0; j < oindices.size(); ++j) {
                mve::CameraInfo const & cam = trajectory[oindices[j]];

                math::Vec3f pos;
                cam.fill_camera_pos(pos.begin());
                math::Matrix4f w2c;
                cam.fill_world_to_cam(w2c.begin());

                {
                    dim3 grid(cacc::divup(num_verts, KERNEL_BLOCK_SIZE));
                    dim3 block(KERNEL_BLOCK_SIZE);
                    update_observation_rays<<<grid, block, 0, stream>>>(
                        true, cacc::Vec3f(pos.begin()), args.max_distance,
                        cacc::Mat4f(w2c.begin()), cacc::Mat3f(calib.begin()),
                        width, height,
                        dbvh_tree->accessor(), dcloud->cdata(),
                        dobs_rays->cdata()
                    );
                }

                cacc::sync(stream, event, std::chrono::microseconds(100));
            }

            /* Calculate total simplex volume - step size. */
            #pragma omp for reduction(+:volume)
            for (std::size_t j = 0; j < simplices.size(); ++j) {
                std::array<math::Vector<float, 3>, 4> & v = simplices[j].verts;
                volume += std::abs(math::geom::tetrahedron_volume(v[0], v[1], v[2], v[3]));
            }

            #pragma omp single
            {
                {
                    dim3 grid(cacc::divup(num_verts, 2));
                    dim3 block(32, 2);
                    process_observation_rays<<<grid, block, 0, stream>>>(
                        dobs_rays->cdata());
                }

                /* Evaluate new reconstructabilities. */
                {
                    dim3 grid(cacc::divup(num_verts, KERNEL_BLOCK_SIZE));
                    dim3 block(KERNEL_BLOCK_SIZE);
                    evaluate_observation_rays<<<grid, block, 0, stream>>>(
                        dobs_rays->cdata(), drecons->cdata());

                    calculate_func_recons<<<grid, block, 0, stream>>>(
                        drecons->cdata(), args.target_recon, dwrecons->cdata());
                }

                cacc::sync(stream, event, std::chrono::microseconds(100));

                //float length = utp::length(trajectory);

                /* Calculate value of objective function. */
                float avg_wrecon = cacc::reduction::sum(dwrecons) / num_verts;
                ovalues.push_back(avg_wrecon);

                /* Terminate if improvement to small. */
                if (i > 0) {
                    float improvement = ovalues[std::max((int)i - 100, 0)] - avg_wrecon;
                    if (improvement < args.target_recon * 1e-4f) {
                        terminate = true;
                    }
                }

                float max_recon = cacc::reduction::max(drecons);
                std::cout << i << " "
                    << oindices.size() << " "
                    << avg_wrecon << " "
                    << volume << std::endl;
            }
        }
        cudaEventDestroy(event);
        cudaStreamDestroy(stream);
    }

    utp::save_trajectory(trajectory, args.out_trajectory);

    return EXIT_SUCCESS;
}
