#ifndef GEOM_ICP_HEADER
#define GEOM_ICP_HEADER

#include <limits>
#include <random>
#include <future>
#include <vector>
#include <cassert>
#include <algorithm>
#include <unordered_set>

#include "math/matrix_tools.h"
#include "math/matrix_svd.h"

#include "acc/bvh_tree.h"

#include "transform.h"

constexpr float eps = std::numeric_limits<float>::epsilon();

typedef std::pair<math::Vec3f, math::Vec3f> Correspondence;
typedef std::vector<Correspondence> Correspondences;

std::vector<uint> choose_random(std::size_t n, std::size_t from, std::size_t to) {
    static std::mt19937 gen;
    std::uniform_int_distribution<uint> dis(from, to);
    std::unordered_set<uint> samples;
    while(samples.size() < n) {
        samples.insert(dis(gen));
    }
    return std::vector<uint>(samples.begin(), samples.end());
}

math::Matrix3f
determine_rotation(std::vector<Correspondence> const & ccorrespondences,
    std::vector<uint> const & samples)
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
    math::Matrix3d VUt = V * U.transposed();
    math::Matrix3d F(0.0f);
    F(0, 0) = 1.0f; F(1, 1) = 1.0f; F(2, 2) = math::matrix_determinant(VUt);
    math::Matrix3f R = V * F * U.transposed();

    return R;
}

math::Matrix3f
determine_rotation(std::vector<Correspondence> const & ccorrespondences,
    std::vector<uint> const & samples, float threshold, std::vector<uint> * inliers)
{
    math::Matrix3f R = determine_rotation(ccorrespondences, samples);

    for (std::size_t j = 0; j < ccorrespondences.size(); ++j) {
        math::Vec3f f, c;
        std::tie(f, c) = ccorrespondences[j];
        if ((R * f - c).norm() > threshold) continue;
        inliers->push_back(j);
    }

    return R;
}

std::pair<math::Matrix3f, uint>
estimate_rotation(std::vector<Correspondence> const & ccorrespondences,
    std::vector<uint> samples, float threshold)
{
    math::Matrix3f R;
    std::vector<uint> inliers;

    R = determine_rotation(ccorrespondences, samples, threshold, &inliers);

    if (inliers.size() < samples.size()) {
        return std::make_pair(R, 0);
    }

    samples.swap(inliers);
    inliers.clear();

    R = determine_rotation(ccorrespondences, samples, threshold, &inliers);

    return std::make_pair(R, inliers.size());
}

math::Matrix4f
estimate_transform(std::vector<Correspondence> const & correspondences, float threshold)
{
    uint max_scale_samples = 10000;
    std::vector<uint> samples;
    samples = choose_random(max_scale_samples, 0, correspondences.size() - 1);

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
    std::size_t n = std::floor(scales.size() * 0.05f);
    float scale = std::accumulate(scales.begin() + n, scales.end() - n, 0.0)
         / (scales.size() - 2 * n);

    std::cout << "  Scale: " << scale << std::endl;

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
                estimate_rotation, std::cref(ccorrespondences), samples, threshold
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
    std::cout << "  RANSAC inliers: " << 100.0f * inliers << "%" << std::endl;

    return T;
}

math::Matrix4f
estimate_transform(mve::TriangleMesh::ConstPtr mesh,
    mve::TriangleMesh::ConstPtr rmesh, uint num_iters, float * avg_dist_ptr)
{
    assert(num_iters < std::numeric_limits<uint>::max() - 1);

    std::vector<math::Vec3f> const & verts = mesh->get_vertices();
    std::vector<uint> const & faces = mesh->get_faces();
    assert(faces.size() > 0);
    acc::BVHTree<uint, math::Vec3f> bvh_tree(faces, verts);

    std::vector<math::Vec3f> const & rverts = rmesh->get_vertices();
    std::vector<uint> const & rfaces = rmesh->get_faces();
    assert(rfaces.size() > 0);
    acc::BVHTree<uint, math::Vec3f> rbvh_tree(rfaces, rverts);

    math::Matrix4f T(0.0f);
    T(0, 0) = T(1, 1) = T(2, 2) = T(3, 3) = 1.0f;
    double avg_dist;

    Correspondences correspondences(verts.size() + rverts.size());
    for (uint i = 0; i < num_iters + 1; ++i) {
        math::Matrix4f Ti = inverse_transform(T);

        avg_dist = 0.0;

        /* Find correspondences */
        #pragma omp parallel for reduction(+:avg_dist)
        for (std::size_t i = 0; i < verts.size(); ++i) {
            math::Vec3f vertex = T.mult(verts[i], 1.0f);
            math::Vec3f cp = rbvh_tree.closest_point(vertex);

            avg_dist += (vertex - cp).norm();
            correspondences[i] = std::make_pair(vertex, cp);
        }

        #pragma omp parallel for reduction(+:avg_dist)
        for (std::size_t i = 0; i < rverts.size(); ++i) {
            math::Vec3f vertex = Ti.mult(rverts[i], 1.0f);
            math::Vec3f cp = bvh_tree.closest_point(vertex);

            avg_dist += (vertex - cp).norm();
            correspondences[verts.size() + i] = std::make_pair(vertex, cp);
        }

        avg_dist /= (verts.size() + rverts.size());

        if (i == 0) break;

        float threshold = avg_dist / 2.0f;
        T = estimate_transform(correspondences, threshold) * T;
    }

    if (avg_dist_ptr != nullptr) *avg_dist_ptr = avg_dist;

    return T;
}

#endif /* GEOM_ICP_HEADER */
