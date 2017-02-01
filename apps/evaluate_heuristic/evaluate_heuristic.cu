#include <fstream>
#include <iostream>

#include "util/system.h"
#include "util/arguments.h"

#include "util/io.h"
#include "util/numpy_io.h"

#include "cacc/util.h"
#include "cacc/math.h"

#include "eval/kernels.h"

#include "stat/correlations.h"

#include "mve/scene.h"
#include "mve/image.h"

struct Arguments {
    std::string scene;
    std::string image;
    std::string gt_mesh;
    std::string file;
    std::string recon_cloud;
    std::string obs_cloud;
    float max_distance;
    float target_recon;
};

typedef unsigned int uint;
typedef acc::BVHTree<uint, math::Vec3f> BVHTree;

Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_minnum(4);
    args.set_nonopt_maxnum(4);
    args.set_usage("Usage: " + std::string(argv[0]) +
        " [OPTS] SCENE IMAGE GT_MESH FILE");
    args.set_description("Evaluates Spearman's rank correlation between "
        "depth error and heuristic for multiple parameter sets.");
    args.add_option('r', "recon-cloud", true,
        "save cloud with predicted reconstructabilities");
    args.add_option('o', "obs-cloud", true,
        "save cloud with number of observations reconstructabilities");
    args.add_option('\0', "max-distance", true, "maximum distance to surface [80.0]");
    args.parse(argc, argv);

    Arguments conf;
    conf.scene = args.get_nth_nonopt(0);
    conf.image = args.get_nth_nonopt(1);
    conf.gt_mesh = args.get_nth_nonopt(2);
    conf.file = args.get_nth_nonopt(3);
    conf.max_distance = 80.0f;

    for (util::ArgResult const* i = args.next_option();
         i != 0; i = args.next_option()) {
        switch (i->opt->sopt) {
        case 'r':
            conf.recon_cloud = i->arg;
        break;
        case 'o':
            conf.obs_cloud = i->arg;
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

template <int N> inline void
patch(mve::FloatImage::Ptr img, int x, int y, float (*ptr)[N][N]) {
    static_assert(N % 2 == 1, "Requires odd patch size");
    constexpr int e = N / 2;
    for (int i = -e; i <= e; ++i) {
        for (int j = -e; j <= e; ++j) {
            (*ptr)[e + j][e + i] = img->at(x + j, y + i, 0);
        }
    }
}

int main(int argc, char **argv) {
    util::system::register_segfault_handler();
    util::system::print_build_timestamp(argv[0]);

    Arguments args = parse_args(argc, argv);

    int device = cacc::select_cuda_device(3, 5);

    mve::Scene::Ptr scene;
    try {
        scene = mve::Scene::create(args.scene);
    } catch (std::exception& e) {
        std::cerr << "Could not open scene: " << e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }

    mve::TriangleMesh::Ptr mesh;
    try {
        mesh = mve::geom::load_ply_mesh(args.gt_mesh);
    } catch (std::exception& e) {
        std::cerr << "\tCould not load mesh: "<< e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }

    std::vector<math::Vec3f> const & vertices = mesh->get_vertices();
    std::vector<uint> const & faces = mesh->get_faces();
    std::cout << "Building BVH... " << std::flush;
    BVHTree::Ptr bvh_tree = BVHTree::create(faces, vertices);
    std::cout << "done." << std::endl;

    std::vector<float> errors;
    std::vector<math::Vec3f> verts;
    std::vector<math::Vec3f> normals;

    std::vector<mve::View::Ptr> views = scene->get_views();
    for (mve::View::Ptr & view : views) {
        if (view == nullptr) continue;
        if (!view->has_image(args.image, mve::IMAGE_TYPE_FLOAT)) {
            std::cerr << "Warning view " << view->get_name()
                << " has no image " << args.image << std::endl;
            continue;
        }

        mve::FloatImage::Ptr dmap = view->get_float_image(args.image);

        mve::CameraInfo const & camera = view->get_camera();
        math::Vec3f origin;
        camera.fill_camera_pos(origin.begin());
        math::Matrix3f invcalib;
        camera.fill_inverse_calibration(invcalib.begin(),
            dmap->width(), dmap->height());
        math::Matrix3f c2w_rot;
        camera.fill_cam_to_world_rot(c2w_rot.begin());

        /* Ignore border - issues with kernel approaches. */
        int border = 0.01f * max(dmap->width(), dmap->height());
        for (int y = border; y < dmap->height() - border; ++y) {
            for (int x = border; x < dmap->width() - border; ++x) {
                float depth = dmap->at(x, y, 0);

                BVHTree::Ray ray;
                ray.origin = origin;
                math::Vec3f v = invcalib *
                    math::Vec3f ((float)x + 0.5f, (float)y + 0.5f, 1.0f);
                ray.dir = c2w_rot.mult(v.normalized()).normalize();
                ray.tmin = 0.0f;
                ray.tmax = std::numeric_limits<float>::infinity();

                /* Ground truth depth? */
                BVHTree::Hit hit;
                if (!bvh_tree->intersect(ray, &hit)) continue;

                verts.push_back(origin + (hit.t * ray.dir));
                math::Vec3f v0 = vertices[faces[hit.idx * 3]];
                math::Vec3f v1 = vertices[faces[hit.idx * 3 + 1]];
                math::Vec3f v2 = vertices[faces[hit.idx * 3 + 2]];
                normals.push_back((v2 - v0).cross(v1 - v0).normalize());

                //float depths[25];
                //patch(dmap, x, y, (float (*)[5][5])&depths);
                //if (std::any_of(depths, depths + 25,
                //        [] (float d) { return d == 0.0f; })) {
                if (depth == 0) {
                    errors.push_back(-1.0f);
                } else {
                    errors.push_back(std::abs(depth - (hit.t * ray.dir).norm()));
                }
            }
        }
    }

    /* Construct cloud for heuristic evaluation on GPU. */
    cacc::PointCloud<cacc::HOST>::Ptr cloud;
    cloud = cacc::PointCloud<cacc::HOST>::create(verts.size());
    cacc::PointCloud<cacc::HOST>::Data data = cloud->cdata();
    for (std::size_t i = 0; i < verts.size(); ++i) {
        data.vertices_ptr[i] = cacc::Vec3f(verts[i].begin());
        data.normals_ptr[i] = cacc::Vec3f(normals[i].begin());
        data.values_ptr[i] = 0.0f;
        data.qualities_ptr[i] = 1.0f;
    }
    cacc::PointCloud<cacc::DEVICE>::Ptr dcloud;
    dcloud = cacc::PointCloud<cacc::DEVICE>::create<cacc::HOST>(cloud);

    cacc::BVHTree<cacc::DEVICE>::Ptr dbvh_tree;
    dbvh_tree = cacc::BVHTree<cacc::DEVICE>::create<uint, math::Vec3f>(bvh_tree);

    uint num_verts = verts.size();
    uint max_cameras = 32;

    cacc::VectorArray<cacc::Vec3f, cacc::DEVICE>::Ptr dobs_rays;
    dobs_rays = cacc::VectorArray<cacc::Vec3f, cacc::DEVICE>::create(num_verts, max_cameras);
    cacc::Array<float, cacc::DEVICE>::Ptr drecons;
    drecons = cacc::Array<float, cacc::DEVICE>::create(num_verts);
    drecons->null();

    int width = 1920;
    int height = 1080;
    math::Matrix4f w2c;
    math::Matrix3f calib;
    math::Vec3f view_pos(0.0f);

    /* Populate view direction histograms. */
    {
        cudaStream_t stream;
        cudaStreamCreate(&stream);
        dim3 grid(cacc::divup(num_verts, KERNEL_BLOCK_SIZE));
        dim3 block(KERNEL_BLOCK_SIZE);

        for (mve::View::Ptr const & view : scene->get_views()) {
            if (view == nullptr) continue;

            mve::CameraInfo cam = view->get_camera();
            cam.fill_calibration(calib.begin(), width, height);
            cam.fill_world_to_cam(w2c.begin());
            cam.fill_camera_pos(view_pos.begin());

            update_observation_rays<<<grid, block, 0, stream>>>(
                true, cacc::Vec3f(view_pos.begin()), args.max_distance,
                cacc::Mat4f(w2c.begin()), cacc::Mat3f(calib.begin()), width, height,
                dbvh_tree->accessor(), dcloud->cdata(), dobs_rays->cdata()
            );
        }

        cudaStreamDestroy(stream);
        CHECK(cudaDeviceSynchronize());
    }

    {
        dim3 grid(cacc::divup(num_verts, 2));
        dim3 block(32, 2);
        process_observation_rays<<<grid, block>>>(
           dobs_rays->cdata());
    }

    std::vector<float> heuristics(verts.size());
    std::vector<float> observations(verts.size());

    float m_k = 8;
    float m_x0 =4;
    float t_k = 32;
    float t_x0 = 16;
    {
        configure_heuristic(m_k, m_x0, t_k, t_x0);
        CHECK(cudaDeviceSynchronize());

        dim3 grid(cacc::divup(num_verts, KERNEL_BLOCK_SIZE));
        dim3 block(KERNEL_BLOCK_SIZE);
        evaluate_observation_rays<<<grid, block>>>(dobs_rays->cdata(),
            drecons->cdata());
        CHECK(cudaDeviceSynchronize());

        {
            cacc::Array<float, cacc::HOST> recons(*drecons);
            cacc::Array<float, cacc::HOST>::Data const & data = recons.cdata();
            CHECK(cudaDeviceSynchronize());

            for (std::size_t k = 0; k < data.num_values; ++k) {
                heuristics[k] = data.data_ptr[k];
            }
        }
        std::cout << stat::spearmans_rank_correlation(heuristics, errors) << std::endl;

        {
            cacc::VectorArray<cacc::Vec3f, cacc::HOST> obs_rays(*dobs_rays);
            cacc::VectorArray<cacc::Vec3f, cacc::HOST>::Data const & data = obs_rays.cdata();
            CHECK(cudaDeviceSynchronize());

            for (std::size_t k = 0; k < data.num_cols; ++k) {
                observations[k] = data.num_rows_ptr[k];
            }
        }

        save_numpy_file(heuristics, errors, observations, args.file);
    }

    if (!args.recon_cloud.empty() || !args.obs_cloud.empty()) {
        mve::TriangleMesh::Ptr mesh = mve::TriangleMesh::create();

        mesh->get_vertices().assign(verts.begin(), verts.end());

        mve::geom::SavePLYOptions opts;
        opts.write_vertex_values = true;

        if (!args.recon_cloud.empty()) {
            mesh->get_vertex_values().assign(heuristics.begin(), heuristics.end());
            mve::geom::save_ply_mesh(mesh, args.recon_cloud, opts);
        }

        if (!args.obs_cloud.empty()) {
            mesh->get_vertex_values().assign(observations.begin(), observations.end());
            mve::geom::save_ply_mesh(mesh, args.obs_cloud, opts);
        }
    }

    return EXIT_SUCCESS;
}
