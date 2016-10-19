#include <random>
#include <future>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <unordered_set>

#include "util/system.h"
#include "util/arguments.h"
#include "util/choices.h"

#include "util/matrix_io.h"

#include "math/matrix_tools.h"
#include "math/matrix_svd.h"

#include "mve/scene.h"
#include "mve/bundle_io.h"
#include "mve/mesh_io_ply.h"

#include "acc/bvh_tree.h"

constexpr float eps = std::numeric_limits<float>::epsilon();

struct Arguments {
    std::string bundle;
    std::string scene;
    std::string mesh;
    std::string transform;
};

Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_minnum(3);
    args.set_nonopt_maxnum(3);
    args.set_usage("Usage: " + std::string(argv[0]) + " [OPTS] BUNDLE SCENE MESH");
    args.set_description("Determine transform of bundle to scene and mesh by "
        "intersecting rays through the 2D feature projections with the mesh "
        "leveraging camera correspondences.");
    args.add_option('t', "transform", true, "save transform to matrix file");
    args.parse(argc, argv);

    Arguments conf;
    conf.bundle = args.get_nth_nonopt(0);
    conf.scene = args.get_nth_nonopt(1);
    conf.mesh = args.get_nth_nonopt(2);

    for (util::ArgResult const* i = args.next_option();
         i != 0; i = args.next_option()) {
        switch (i->opt->sopt) {
        case 't':
            conf.transform = i->arg;
        break;
        default:
            throw std::invalid_argument("Invalid option");
        }
    }

    return conf;
}

typedef std::pair<math::Vec3f, math::Vec3f> Correspondence;

static std::mt19937 gen;

std::vector<uint> choose_random(std::size_t n, std::size_t from, std::size_t to) {
    std::uniform_int_distribution<uint> dis(from, to);
    std::unordered_set<uint> samples;
    while(samples.size() < n) {
        samples.insert(dis(gen));
    }
    return std::vector<uint>(samples.begin(), samples.end());
}

math::Matrix3f
determine_rotation(std::vector<Correspondence> const & ccorrespondences,
    std::vector<uint> const & samples, std::vector<uint> * inliers)
{
    math::Matrix3d cov(0.0f);

    /* Calculate covariance matrix. */
    for (std::size_t j = 0; j < samples.size(); ++j) {
        math::Vec3f f, c;
        std::tie(f, c) = ccorrespondences[samples[j]];
        cov += math::Matrix<float, 3, 1>(f.begin()) *
            math::Matrix<float, 1, 3>(c.begin());
    }
    cov /= samples.size();

    /* Estimate rotation and translation */
    math::Matrix3d U, S, V;
    math::matrix_svd(cov, &U, &S, &V);
    math::Matrix3f R = V * U.transposed();
    if (std::abs(math::matrix_determinant(R) - 1.0f) > eps) {
        return R;
    }

    for (std::size_t j = 0; j < ccorrespondences.size(); ++j) {
        math::Vec3f f, c;
        std::tie(f, c) = ccorrespondences[j];
        //TODO threshold
        if ((R * f - c).norm() > 0.01f || inliers == nullptr) continue;
        inliers->push_back(j);
    }

    return R;
}

std::pair<math::Matrix3f, uint>
determine_robust_rotation(std::vector<Correspondence> const & ccorrespondences,
    std::vector<uint> samples)
{
    math::Matrix3f R;
    std::vector<uint> inliers;

    inliers.clear();
    R = determine_rotation(ccorrespondences, samples, &inliers);

    if (inliers.size() < samples.size()) {
        return std::make_pair(R, 0);
    }

    samples.swap(inliers);

    inliers.clear();
    R = determine_rotation(ccorrespondences, samples, &inliers);

    return std::make_pair(R, inliers.size());
}

math::Matrix4f
determine_transform(std::vector<Correspondence> const & correspondences)
{
    uint max_scale_samples = 10000;
    std::vector<uint> samples;
    if (correspondences.size() < max_scale_samples) {
        samples.resize(correspondences.size());
        std::iota(samples.begin(), samples.end(), 0);
    } else {
        samples = choose_random(max_scale_samples, 0, correspondences.size() - 1);
    }

    /* Calculate scales based on pairwise distances. */
    std::vector<double> scales;
    scales.reserve(samples.size());
    for (std::size_t i = 0; i < samples.size() - 1; ++i) {
        math::Vec3f f1, f2, c1, c2;
        std::tie(f1, c1) = correspondences[samples[i]];
        std::tie(f2, c2) = correspondences[samples[i + 1]];
        float dist = (c1 - c2).norm();

        if (dist < eps) continue;

        scales.push_back((f1 - f2).norm() / dist);
    }

    /* Calculate scale as mean from "inlier" values. */
    std::sort(scales.begin(), scales.end());
    uint n = std::floor(scales.size() * 0.05f);
    float scale = std::accumulate(scales.begin() + n, scales.end() - n, 0.0)
         / (scales.size() - 2 * n);

    std::cout << "\tScale: " << scale << std::endl;

    /* Calculate centroids. */
    math::Vec3f centroid1(0.0f), centroid2(0.0f);
    for (std::size_t i = 0; i < samples.size(); ++i) {
        math::Vec3f f, c;
        std::tie(f, c) = correspondences[samples[i]];
        centroid1 += f;
        centroid2 += c;
    }
    centroid1 /= samples.size();
    centroid2 /= samples.size();

    std::vector<Correspondence> ccorrespondences(correspondences.size());
    for (std::size_t i = 0; i < correspondences.size(); ++i) {
        math::Vec3f f, c;
        std::tie(f, c) = correspondences[i];
        ccorrespondences[i].first = f - centroid1;
        ccorrespondences[i].second = scale * (c - centroid2);
    }

    uint ransac_iterations = 1000;

    std::vector<std::future<std::pair<math::Matrix3f, uint> > > futures;
    for (uint i = 0; i < ransac_iterations; ++i) {
        std::vector<uint> samples = choose_random(3, 0, ccorrespondences.size() - 1);
        futures.push_back(std::async(std::launch::async,
                determine_robust_rotation, std::cref(ccorrespondences), samples
            )
        );
    }

    uint max_inliers = 0;
    math::Matrix4f T;
    math::matrix_set_identity(&T);
    for (std::size_t i = 0; i < futures.size(); ++i) {
        math::Matrix3f R;
        uint num_inliers;
        std::tie(R, num_inliers) = futures[i].get();

        if (num_inliers > max_inliers) {
            max_inliers = num_inliers;
            math::Vec3f t = -R * centroid1 + scale * centroid2;
            math::Vec4f u4(0.0f, 0.0f, 0.0f, 1.0f);
            T = (R / scale).hstack(t / scale).vstack(u4);
        }
    }

    float inliers = max_inliers / static_cast<float>(ccorrespondences.size());
    std::cout << "\tRANSAC inliers: " << 100.0f * inliers << "%" << std::endl;

    return T;
}

int main(int argc, char **argv) {
    util::system::register_segfault_handler();
    util::system::print_build_timestamp(argv[0]);

    Arguments args = parse_args(argc, argv);

    mve::Scene::Ptr scene;
    try {
        scene = mve::Scene::create(args.scene);
    } catch (std::exception& e) {
        std::cerr << "Could not open scene: " << e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }

    mve::TriangleMesh::Ptr mesh;
    try {
        mesh = mve::geom::load_ply_mesh(args.mesh);
    } catch (std::exception& e) {
        std::cerr << "\tCould not load mesh: "<< e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }

    mve::Bundle::Ptr bundle;
    try {
        bundle = mve::load_mve_bundle(args.bundle);
    } catch (std::exception& e) {
        std::cerr << "\tCould not load bundle: "<< e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }

    std::vector<math::Vec3f> const & vertices = mesh->get_vertices();
    std::vector<uint> const & faces = mesh->get_faces();
    acc::BVHTree<uint, math::Vec3f> bvh_tree(faces, vertices);

    std::vector<mve::View::Ptr> const & views = scene->get_views();

    std::vector<mve::CameraInfo> cameras(views.size());
    for (std::size_t i = 0; i < cameras.size(); ++i) {
        mve::View::Ptr const & view = views[i];
        if (view == nullptr) continue;

        cameras[i] = view->get_camera();
    }

    int width = 1920;
    int height = 1080;

    mve::Bundle::Features & features = bundle->get_features();
    mve::Bundle::Cameras & bundle_cameras = bundle->get_cameras();

    #pragma omp parallel for
    for (std::size_t i = 0; i < features.size(); ++i) {
        math::Vec3f feature = math::Vec3f(features[i].pos);
        std::vector<mve::Bundle::Feature2D> & refs = features[i].refs;
        for (mve::Bundle::Feature2D & feature2d : refs) {
            mve::CameraInfo const & cam = bundle_cameras[feature2d.view_id];

            math::Vec3f cam_pos;
            math::Matrix4f w2c;
            math::Matrix3f calib;
            cam.fill_camera_pos(cam_pos.begin());
            cam.fill_calibration(calib.begin(), width, height);
            cam.fill_world_to_cam(w2c.begin());
            math::Vec3f pt = calib * w2c.mult(feature, 1.0f);
            math::Vec2f pos(pt[0] / pt[2], pt[1] / pt[2]);

            std::copy(pos.begin(), pos.end(), feature2d.pos);
        }
    }

    std::vector<Correspondence> correspondences(features.size());

    #pragma omp parallel for
    for (std::size_t i = 0; i < features.size(); ++i) {
        math::Vec3f feature = math::Vec3f(features[i].pos);
        std::vector<math::Vec3f> projections;

        std::vector<mve::Bundle::Feature2D> const & refs = features[i].refs;
        for (mve::Bundle::Feature2D const & feature2d : refs) {
            mve::CameraInfo const & cam = cameras[feature2d.view_id];
            math::Vec2f pos(feature2d.pos);

            math::Vec3f cam_pos;
            math::Matrix3f invcalib;
            math::Matrix3f c2w_rot;
            cam.fill_camera_pos(cam_pos.begin());
            cam.fill_inverse_calibration(invcalib.begin(), width, height);
            cam.fill_cam_to_world_rot(c2w_rot.begin());

            math::Vec3f v = invcalib * math::Vec3f(pos[0], pos[1], 1.0f);

            acc::Ray<math::Vec3f> ray;
            ray.origin = cam_pos;
            ray.dir = c2w_rot.mult(v.normalized()).normalize();
            ray.tmin = 0.0f;
            ray.tmax = std::numeric_limits<float>::infinity();

            acc::BVHTree<uint, math::Vec3f>::Hit hit;
            if (!bvh_tree.intersect(ray, &hit)) continue;

            projections.push_back(cam_pos + hit.t * ray.dir);
        }

        if (projections.empty()) continue;
        //TODO delete correspondences...

        for (int j = 0; j < 3; ++j) {
            std::stable_sort(projections.begin(), projections.end(),
                [j] (math::Vec3f const & r, math::Vec3f const & l) {
                    return r[j] > l[j];
                }
            );
        }
        math::Vec3f projection = projections[projections.size() / 2];

        correspondences[i] = std::make_pair(feature, projection);
    }

    math::Matrix4f T;

    std::cout << "Estimating transform based on feature correspondences." << std::endl;
    T = determine_transform(correspondences);

    #pragma omp parallel for
    for (std::size_t i = 0; i < features.size(); ++i) {
        math::Vec3f feature = math::Vec3f(features[i].pos);
        math::Vec3f vertex = T.mult(feature, 1.0f);

        math::Vec3f projection = bvh_tree.closest_point(vertex);

        correspondences[i] = std::make_pair(vertex, projection);
    }

    std::cout << "Refining transform based on closest points." << std::endl;
    T = determine_transform(correspondences) * T;

    double ssd = 0.0f;
    for (std::size_t i = 0; i < features.size(); ++i) {
        math::Vec3f feature = math::Vec3f(features[i].pos);
        math::Vec3f vertex = T.mult(feature, 1.0f);

        math::Vec3f projection = bvh_tree.closest_point(vertex);

        ssd += (vertex - projection).norm();
    }
    std::cout << "Average distance to surface: " << ssd / features.size() << std::endl;

    if (!args.transform.empty()) {
        save_matrix_to_file(T, args.transform);
    } else {
        std::cout << T << std::endl;
    }

    return EXIT_SUCCESS;
}
