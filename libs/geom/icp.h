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
    while(samples.size() < std::min(n, to - from)) {
        samples.insert(dis(gen));
    }
    return std::vector<uint>(samples.begin(), samples.end());
}

template <typename T, int N>
T trace(math::Matrix<T, N, N> mat) {
    T ret = T(0.0);
    for (int i = 0; i < N; ++i) {
        ret += mat(i, i);
    }
    return ret;
}

std::pair<math::Matrix3f, float>
determine_rotation_and_scale(math::Matrix3d const & cov, double sigma2) {
    math::Matrix3d U, S, V;
    math::matrix_svd(cov, &U, &S, &V);
    math::Matrix3d VUt = V * U.transposed();
    double det = math::matrix_determinant(VUt);
    if (det > 0.0) {
        double scale = trace(S) / sigma2;
        return {VUt, scale};
    } else {
        math::Matrix3d F(0.0);
        F(0, 0) = 1.0; F(1, 1) = 1.0; F(2, 2) = -1.0;
        double scale = trace(S * F) / sigma2;
        return {V * F * U.transposed(), scale};
    }
}

std::pair<math::Matrix3f, float>
determine_rotation_and_scale(std::vector<Correspondence> const & ccorrespondences)
{
    math::Matrix3d cov(0.0f);
    double sigma2 = 0.0f;

    for (std::size_t i = 0; i < ccorrespondences.size(); ++i) {
        math::Vec3f f, c;
        std::tie(f, c) = ccorrespondences[i];
        cov += math::Matrix<float, 3, 1>(f.begin()) *
            math::Matrix<float, 1, 3>(c.begin());
        sigma2 += f.square_norm();
    }
    cov /= ccorrespondences.size();
    sigma2 /= ccorrespondences.size();

    return determine_rotation_and_scale(cov, sigma2);
}

std::pair<math::Matrix3f, float>
determine_rotation_and_scale(std::vector<Correspondence> const & ccorrespondences,
    std::vector<uint> const & samples)
{
    math::Matrix3d cov(0.0f);
    double sigma2 = 0.0f;

    for (std::size_t j = 0; j < samples.size(); ++j) {
        math::Vec3f f, c;
        std::tie(f, c) = ccorrespondences[samples[j]];
        cov += math::Matrix<float, 3, 1>(f.begin()) *
            math::Matrix<float, 1, 3>(c.begin());
        sigma2 += f.square_norm();
    }
    cov /= samples.size();
    sigma2 /= samples.size();

    return determine_rotation_and_scale(cov, sigma2);
}

std::pair<math::Matrix3f, float>
determine_rotation_and_scale(std::vector<Correspondence> const & ccorrespondences,
    std::vector<uint> const & samples, float threshold, std::vector<uint> * inliers)
{
    math::Matrix3f R;
    float s;
    std::tie(R, s) = determine_rotation_and_scale(ccorrespondences, samples);

    for (std::size_t j = 0; j < ccorrespondences.size(); ++j) {
        math::Vec3f f, c;
        std::tie(f, c) = ccorrespondences[j];
        if (((R * s * f) - c).norm() > threshold) continue;
        inliers->push_back(j);
    }

    return {R, s};
}

std::tuple<math::Matrix3f, float, uint>
estimate_rotation(std::vector<Correspondence> const & ccorrespondences,
    std::vector<uint> samples, float threshold)
{
    math::Matrix3f R;
    std::vector<uint> inliers;

    float s;
    std::tie(R, s) = determine_rotation_and_scale(ccorrespondences,
        samples, threshold, &inliers);

    if (inliers.size() < samples.size()) {
        return std::make_tuple(R, 1.0f, 0u);
    }

    samples.swap(inliers);
    inliers.clear();

    std::tie(R, s) = determine_rotation_and_scale(ccorrespondences,
        samples, threshold, &inliers);

    return std::make_tuple(R, s, inliers.size());
}

std::pair<math::Vec3f, math::Vec3f>
center(std::vector<Correspondence> * correspondences) {
    math::Vec3d c0(0.0f), c1(0.0f);
    for (std::size_t i = 0; i < correspondences->size(); ++i) {
        math::Vec3f f, c;
        std::tie(f, c) = correspondences->at(i);
        c0 += f;
        c1 += c;
    }
    c0 /= correspondences->size();
    c1 /= correspondences->size();

    for (std::size_t i = 0; i < correspondences->size(); ++i) {
        math::Vec3f f, c;
        std::tie(f, c) = correspondences->at(i);
        correspondences->at(i).first = f - c0;
        correspondences->at(i).second = c - c1;
    }

    return std::make_pair(c0, c1);
}

math::Matrix4f
estimate_transform(std::vector<Correspondence> correspondences, float threshold)
{
    math::Vec3f c0, c1;
    std::tie(c0, c1) = center(&correspondences);

    std::vector<std::future<std::tuple<math::Matrix3f, float, uint> > > futures;
    for (int i = 0; i < 1000; ++i) {
        std::vector<uint> samples = choose_random(3, 0, correspondences.size() - 1);
        futures.push_back(std::async(std::launch::async,
                estimate_rotation, std::cref(correspondences), samples, threshold
            )
        );
    }

    uint max_inliers = 0;
    math::Matrix4f T;
    math::matrix_set_identity(&T);
    for (std::size_t i = 0; i < futures.size(); ++i) {
        math::Matrix3f R;
        float s;
        uint num_inliers;
        std::tie(R, s, num_inliers) = futures[i].get();

        if (num_inliers > max_inliers) {
            max_inliers = num_inliers;
            math::Vec3f t = c1 + (-R * s * c0);
            T = assemble_transform(R, s, t);
        }
    }

    return T;
}

math::Matrix4f
estimate_transform(std::vector<Correspondence> correspondences)
{
    if (correspondences.empty()) throw std::runtime_error("No correspondences");

    math::Vec3f c0, c1;
    std::tie(c0, c1) = center(&correspondences);

    math::Matrix3f R;
    float s;
    std::tie(R, s) = determine_rotation_and_scale(correspondences);
    math::Vec3f t = c1 + (-R * s * c0);

    return assemble_transform(R, s, t);
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
    double prev_avg_dist, avg_dist = std::numeric_limits<double>::max();

    Correspondences correspondences(verts.size() + rverts.size());
    for (uint i = 0; i < num_iters + 1; ++i) {
        math::Matrix4f Ti = inverse_transform(T);

        prev_avg_dist = avg_dist;
        avg_dist = 0.0;

        /* Find correspondences */
        #pragma omp parallel for reduction(+:avg_dist)
        for (std::size_t j = 0; j < verts.size(); ++j) {
            math::Vec3f vertex = Ti.mult(verts[j], 1.0f);
            math::Vec3f cp = rbvh_tree.closest_point(vertex);

            avg_dist += (vertex - cp).norm();
            correspondences[j] = std::make_pair(vertex, cp);
        }

        #pragma omp parallel for reduction(+:avg_dist)
        for (std::size_t j = 0; j < rverts.size(); ++j) {
            math::Vec3f vertex = T.mult(rverts[j], 1.0f);
            math::Vec3f cp = bvh_tree.closest_point(vertex);

            avg_dist += (vertex - cp).norm();
            correspondences[verts.size() + j] = std::make_pair(vertex, cp);
        }

        avg_dist /= (verts.size() + rverts.size());
        double improvement = prev_avg_dist - avg_dist;

        if (improvement < 1e-5f || i == num_iters) break;

        T = estimate_transform(correspondences) * T;
    }

    if (avg_dist_ptr != nullptr) *avg_dist_ptr = avg_dist;

    return T;
}

#endif /* GEOM_ICP_HEADER */
