/*
 * Copyright (C) 2016-2018, Nils Moehrle
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#ifndef GEOM_PLANE_ESTIMATION_HEADER
#define GEOM_PLANE_ESTIMATION_HEADER

#include "math/vector.h"
#include "math/plane.h"
#include "math/matrix_svd.h"

#include "acc/primitives.h"

math::Vec3f orthogonal(math::Vec3f const & vec) {
    math::Vec3f const n0(1.0f, 0.0f, 0.0f);
    math::Vec3f const n1(0.0f, 1.0f, 0.0f);
    if (std::abs(n0.dot(vec)) < std::abs(n1.dot(vec))) {
        return n0.cross(vec);
    } else {
        return n1.cross(vec);
    }
}

math::Plane3f
least_squares_plane(std::vector<math::Vec3f> const & vertices,
    std::vector<std::size_t> const & indices)
{
    if (indices.size() < 3) {
        throw std::invalid_argument("Expecting three or more points");
    }

    /* Calculate centroid. */
    math::Vec3d c(0.0f);
    for (std::size_t i = 0; i < indices.size(); ++i) {
        c += vertices[indices[i]];
    }
    c /= double(indices.size());

    math::Matrix3d cov(0.0);

    /* Calculate covariance matrix. */
    for (std::size_t i = 0; i < indices.size(); ++i) {
        math::Vec3f const & v = vertices[indices[i]];
        cov(0, 0) += (v[0] - c[0]) * (v[0] - c[0]);
        cov(1, 1) += (v[1] - c[1]) * (v[1] - c[1]);
        cov(2, 2) += (v[2] - c[2]) * (v[2] - c[2]);
        cov(0, 1) += (v[0] - c[0]) * (v[1] - c[1]);
        cov(0, 2) += (v[0] - c[0]) * (v[2] - c[2]);
        cov(1, 2) += (v[1] - c[1]) * (v[2] - c[2]);
    }

    cov(1, 0) = cov(0, 1);
    cov(2, 0) = cov(0, 2);
    cov(2, 1) = cov(1, 2);

    math::Matrix3d V;
    math::matrix_svd<double, 3, 3>(cov, nullptr, nullptr, &V);
    math::Vec3f normal = V.col(2);

    /* Select normal with positive z-direction. */
    if (normal[2] < 0.0)
        normal *= -1.0;

    return math::Plane3f(normal, c);
}

math::Plane3f
estimate_ground_plane(mve::TriangleMesh::ConstPtr cloud, acc::AABB<math::Vec3f> aabb) {
    float threshold = (aabb.max - aabb.min).norm() * 1e-5f;

    std::vector<math::Vec3f> const & verts = cloud->get_vertices();

    std::vector<std::size_t> inliers;

    std::default_random_engine gen;
    std::uniform_int_distribution<std::size_t> dis(0, verts.size());

    /* Estimate plane via RANSAC. */
    #pragma omp parallel
    {
        std::vector<std::size_t> current_inliers;
        current_inliers.reserve(verts.size());

        #pragma omp for
        for (std::size_t iteration = 0; iteration < 1000; ++iteration) {
            current_inliers.clear();

            math::Vec3f v1, v2, v3;
            #pragma omp critical
            {
                v1 = verts[dis(gen)];
                v2 = verts[dis(gen)];
                v3 = verts[dis(gen)];
            }
            math::Plane3f plane(v1, v2, v3);

            for (std::size_t i = 0; i < verts.size(); ++i) {
                if (std::abs(plane.point_dist(verts[i])) > threshold) continue;
                current_inliers.push_back(i);
            }

            #pragma omp critical
            if (current_inliers.size() > inliers.size()) {
                std::swap(current_inliers, inliers);
            }
        }
    }

    math::Plane3f plane = least_squares_plane(verts, inliers);

    /* Use linear weights on band around plane to determine orientation. */
    threshold = (aabb.max - aabb.min).norm() * 1e-3f;
    float avg_dist = 0.0f;
    for (std::size_t i = 0; i < verts.size(); ++i) {
        float dist = plane.point_dist(verts[i]);
        if (std::abs(dist) < threshold) {
            avg_dist += dist;
        }
    }

    if (avg_dist > 0.0f) {
        plane = plane.invert();
    }

    return plane;
}

math::Plane3f
estimate_ground_plane(mve::TriangleMesh::ConstPtr cloud) {
    std::vector<math::Vec3f> const & verts = cloud->get_vertices();

    constexpr float pi = std::acos(-1.0f);

    std::default_random_engine gen;
    std::uniform_int_distribution<std::size_t> dis(0, verts.size() - 1);

    constexpr int ires = 1<<8;
    constexpr int jres = 2<<8;
    int accum[ires][jres] = {0};

    /* Estimate normal with random Hough transform. */
    for (std::size_t v = 0; v < verts.size(); ++v) {
        math::Vec3f v0, v1, v2;

        v0 = verts[dis(gen)];
        v1 = verts[dis(gen)];
        v2 = verts[dis(gen)];

        math::Vec3f v01 = v1 - v0;
        math::Vec3f v02 = v2 - v0;

        math::Vec3f normal = v01.cross(v02).normalize();

        if (std::isnan(normal.square_norm())) continue;

        if (normal[2] < 0.0f) normal = -normal;

        float theta = std::acos(normal[2]);
        float phi = std::atan2(normal[1], normal[0]) + pi;

        int i = theta / pi * (ires - 1);
        int j = phi / (2.0f * pi) * (jres - 1);

        accum[i][j] += 1;
    }

    /* Find maximum with local support. */
    float theta = 0;
    float phi = 0;
    int max = 0;
    for (int i = 0; i < ires; ++i) {
        for (int j = 0; j < jres; ++j) {
            int sum = 0;
            for (int ri = -1; ri <= 1; ++ri) {
                for (int rj = -1; rj <= 1; ++rj) {
                    sum += accum[(i + ri) & (ires - 1)][(j + rj) & (jres - 1)];
                }
            }

            if (sum > max) {
                max = sum;
                theta = (static_cast<float>(i) / (ires - 1)) * pi;
                phi = (static_cast<float>(j) / (jres - 1)) * 2.0f * pi - pi;
            }
        }
    }

    math::Vec3f normal;
    normal[0] = std::sin(theta) * std::cos(phi);
    normal[1] = std::sin(theta) * std::sin(phi);
    normal[2] = std::cos(theta);

    math::Plane3f plane(normal, math::Vec3f(0.0f));

    std::vector<float> dists(verts.size());
    for (std::size_t i = 0; i < verts.size(); ++i) {
        dists[i] = plane.point_dist(verts[i]);
    }

    auto nth = dists.begin() + dists.size() / 20;
    std::nth_element(dists.begin(), nth, dists.end());

    return math::Plane3f(normal, normal * *nth);
}

#endif /* GEOM_PLANE_ESTIMATION_HEADER */
