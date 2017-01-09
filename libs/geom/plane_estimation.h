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
    float threshold = (aabb.max - aabb.min).norm() * 1e-4f;

    std::vector<math::Vec3f> const & verts = cloud->get_vertices();

    std::vector<std::size_t> inliers;

    std::default_random_engine gen;
    std::uniform_int_distribution<std::size_t> dis(0, verts.size());

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

    uint above = 0;
    uint below = 0;
    for (std::size_t i = 0; i < verts.size(); ++i) {
        if (plane.point_dist(verts[i]) > 0.0f) {
            above += 1;
        } else {
            below += 1;
        }
    }

    if (above < below) {
        plane = plane.invert();
    }

    return plane;
}

#endif /* GEOM_PLANE_ESTIMATION_HEADER */
