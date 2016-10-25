#include <iostream>

#include "util/system.h"
#include "util/arguments.h"
#include "util/file_system.h"

#include "mve/camera.h"
#include "mve/mesh_io_ply.h"

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
    std::string proxy_mesh;
    std::string proxy_cloud;
    std::string out_trajectory;
    uint num_views;
    float min_distance;
    float max_distance;
    float min_altitude;
    float max_altitude;
    float max_velocity;
    float focal_length;
};

Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_minnum(3);
    args.set_nonopt_maxnum(3);
    args.set_usage("Usage: " + std::string(argv[0])
        + " [OPTS] PROXY_MESH PROXY_CLOUD OUT_TRAJECTORY");
    args.set_description("Plans a trajectory maximizing reconstructability");
    args.add_option('\0', "min-distance", true, "minimum distance to surface [0.0]");
    args.add_option('\0', "max-distance", true, "maximum distance to surface [80.0]");
    args.add_option('\0', "min-altitude", true, "minimum altitude [0.0]");
    args.add_option('\0', "max-altitude", true, "maximum altitude [100.0]");
    args.add_option('\0', "max-velocity", true, "maximum velocity [5.0]");
    args.add_option('\0', "focal-length", true, "camera focal length [0.86]");
    args.add_option('n', "num-views", true, "number of views [500]");
    args.parse(argc, argv);

    Arguments conf;
    conf.proxy_mesh = args.get_nth_nonopt(0);
    conf.proxy_cloud = args.get_nth_nonopt(1);
    conf.out_trajectory = args.get_nth_nonopt(2);
    conf.min_distance = 0.0f;
    conf.max_distance = 80.0f;
    conf.min_altitude = 0.0f;
    conf.max_altitude = 100.0f;
    conf.max_velocity = 5.0f;
    conf.num_views = 500;
    conf.focal_length = 0.86f;

    for (util::ArgResult const* i = args.next_option();
         i != 0; i = args.next_option()) {
        switch (i->opt->sopt) {
        case 'n':
            conf.num_views = i->get_arg<uint>();
        break;
        case '\0':
            if (i->opt->lopt == "focal-length") {
                conf.focal_length = i->get_arg<float>();
            } else if (i->opt->lopt == "min-distance") {
                conf.min_distance = i->get_arg<float>();
            } else if (i->opt->lopt == "max-distance") {
                conf.max_distance = i->get_arg<float>();
            } else if (i->opt->lopt == "min-altitude") {
                conf.min_altitude = i->get_arg<float>();
            } else if (i->opt->lopt == "max-altitude") {
                conf.max_altitude = i->get_arg<float>();
            } else if (i->opt->lopt == "max-velocity") {
                conf.max_velocity = i->get_arg<float>();
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
    drecons->null();

    mve::CameraInfo cam;
    cam.flen = args.focal_length;
    math::Matrix3f calib;
    int width = 1920;
    int height = 1080;
    cam.fill_calibration(calib.begin(), width, height);

    struct State {
        math::Vec3f pos;
        math::Vec3f vel;
    } state;

    state.pos = math::Vec3f(-10.0f, 10.0f, 2.0f);
    state.vel = math::Vec3f(1.0f, 0.0f, 0.0f) * 0.1f;

    std::vector<mve::CameraInfo> trajectory;
    struct ViewCandidate {
        math::Vec3f pos;
        math::Matrix3f rot;
    };
    std::array<ViewCandidate, 27> view_candidates;
    std::array<float, 27> view_scores;
    std::mt19937 gen(12345);

    std::array<math::Vec3f, 27> offsets;
    std::array<float, 27> oweights;
    for (int z = 0; z < 3; ++z) {
        for (int y = 0; y < 3; ++y) {
            for (int x = 0; x < 3; ++x) {
                float r = 1.0f + (z - 1) * 0.25f;
                float phi = (y - 1) * (pi / 8.0f);
                float theta = pi / 2 + (x - 1) * (pi / 8.0f);

                int idx = (z * 3 + y) * 3 + x;
                offsets[idx][0] = r * std::sin(theta) * std::cos(phi);
                offsets[idx][1] = r * std::sin(theta) * std::sin(phi);
                offsets[idx][2] = r * std::cos(theta);

                oweights[idx] = 1.0f - (math::Vec3f(1.0f, 0.0f, 0.0f) - offsets[idx]).norm() / 10.0f;
            }
        }
    }

#if 0
    mve::TriangleMesh::Ptr mesh = mve::TriangleMesh::create();
    std::vector<math::Vec3f> & verts = mesh->get_vertices();
    std::vector<float> & values = mesh->get_vertex_values();
    verts.resize(27);
    std::copy(offsets.begin(), offsets.end(), verts.data());
    values.resize(27);
    std::copy(oweights.begin(), oweights.end(), values.data());
    mve::geom::save_ply_mesh(mesh, "/tmp/test.ply");
#endif

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

        for (uint i = 0; i < args.num_views; ++i) {
            #pragma omp for schedule(dynamic)
            for (int j = 0; j < 27; ++j) {
                view_scores[j] = 0.0f;
                dcon_hist->null();

                float penalties = 0.0f;

                //Disable velocity adjustment
                //if (j < 9 || 18 <= j) continue;

                //Disable altitude adjustment
                //if ((j % 3) != 1) continue;

                math::Vec3f & pos = view_candidates[j].pos;
                math::Vec3f rel_offset(0.0f);
                {
                    math::Vec3f rx = state.vel / state.vel.norm();

                    math::Vec3f up = math::Vec3f(0.0f, 0.0f, 1.0f);
                    bool stable = abs(up.dot(rx)) < 0.99f;

                    math::Vec3f ry = up.cross(rx).normalize();
                    math::Vec3f rz = rx.cross(ry).normalize();

                    math::Vec3f const & o = offsets[j];

                    for (int k = 0; k < 3; ++k) {
                        rel_offset[k] = rx[k] * o[0] + ry[k] * o[1] + rz[k] * o[2];
                    }
                }
                pos = state.pos + rel_offset * state.vel.norm();

                if ((pos - state.pos).norm() > args.max_velocity) continue;
                if (pos[2] < args.min_altitude || args.max_altitude < pos[2]) continue;
                std::pair<uint, float> nn;
                if (kd_tree->find_nn(pos, &nn, 2.0f * args.min_distance)) {
                    if (nn.second < args.min_distance) continue;
                    penalties += 1.0f - (nn.second - args.min_distance) / args.min_distance;
                }

                if (penalties > 1.0f) continue;

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
                hist->sync();

                cacc::Image<float, cacc::HOST>::Data data = hist->cdata();

                //TODO write a kernel to select best viewing direction

                float max = 0.0f;
                float theta = 0.0f;
                float phi = 0.0f;
                for (int y = 0; y < data.height; ++y) {
                    for (int x = 0; x < data.width; ++x) {
                        float v = data.data_ptr[y * data.pitch / sizeof(float) + x];
                        if (v > max) {
                            max = v;
                            theta = (x / (float) data.width) * 2.0f * pi;
                            //float theta = (y / (float) data.height) * pi;
                            phi = (0.5f + (y / (float) data.height) / 2.0f) * pi;
                        }
                    }
                }

                view_scores[j] = max * oweights[j] * (1.0f - penalties);

                view_candidates[j].rot = utp::rotation_from_spherical(theta, phi);
            }

            auto it = std::max_element(view_scores.begin(), view_scores.end());
            if (*it < 0.1f) break;

            #pragma omp single
            {
                std::size_t idx = std::distance(view_scores.begin(), it);

                //std::discrete_distribution<> dist(view_scores.begin(), view_scores.end());
                //idx = dist(gen);

                ViewCandidate const & view = view_candidates[idx];

                state.vel = view.pos - state.pos;
                state.pos = view.pos;
                math::Vec3f trans = -view.rot * view.pos;

                std::copy(trans.begin(), trans.end(), cam.trans);
                std::copy(view.rot.begin(), view.rot.end(), cam.rot);

                math::Matrix4f w2c;
                cam.fill_world_to_cam(w2c.begin());
                {
                    dim3 grid(cacc::divup(num_verts, KERNEL_BLOCK_SIZE));
                    dim3 block(KERNEL_BLOCK_SIZE);
                    populate_direction_histogram<<<grid, block, 0, stream>>>(
                        cacc::Vec3f(state.pos.begin()), args.max_distance,
                        cacc::Mat4f(w2c.begin()), cacc::Mat3f(calib.begin()), width, height,
                        dbvh_tree->cdata(), dcloud->cdata(), drecons->cdata(), ddir_hist->cdata()
                    );
                }
                cudaStreamSynchronize(stream);

                float avg_recon = cacc::sum(drecons) / num_verts;
                std::cout << i << " " << avg_recon << std::endl;

                trajectory.push_back(cam);
            }
        }
    }

    utp::save_trajectory(trajectory, args.out_trajectory);

    return EXIT_SUCCESS;
}
