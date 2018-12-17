/*
 * Copyright (C) 2016-2018, Nils Moehrle
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#ifndef UTP_BSPLINE_HEADER
#define UTP_BSPLINE_HEADER

#include <vector>

#include <type_traits>

#include <Eigen/SparseCore>
#include <Eigen/SparseQR>

#include "math/vector.h"

#include "defines.h"

UTP_NAMESPACE_BEGIN

typedef unsigned uint;

template <typename T, uint N, uint p>
class BSpline {
private:
    std::vector<math::Vector<T, N> > points;
    std::vector<T> us;
public:
    BSpline() {
        static_assert(std::is_floating_point<T>::value,
            "Requires floating point type"
        );
    }

    BSpline(std::vector<math::Vector<T, N> > const & verts,
        std::vector<T> knots) : points(verts), us(knots) {}

    static std::vector<T>
    generate_ts(std::vector<math::Vector<T, N> > const & verts) {
        std::size_t n = verts.size();
        std::vector<T> ts(n);

        /* Chord length ts */
        std::vector<T> psum(n, 0.0);
        for(std::size_t i = 1; i < n; ++i) {
            psum[i] = psum[i - 1] + (verts[i - 1] - verts[i]).norm();
        }
        for(std::size_t i = 0; i < n; ++i) {
            ts[i] = psum[i] / psum[n - 1];
        }

        return ts;
    }

    T fit(std::vector<math::Vector<T, N> > const & verts) {
        typedef Eigen::SparseMatrix<T> SpMat;
        typedef Eigen::Triplet<T, int> Triplet;

        std::size_t n = verts.size();
        std::vector<T> ts = generate_ts(verts);

        /* Generate knot vector */
        us.resize(n + p + 1, 0.0);
        for (uint i = 0; i <= p; ++i) {
            //us[i] = T(0.0);
            us[n + i] = T(1.0);
        }

        for (std::size_t i = 1; i < n - p; ++i) {
            for (std::size_t j = i; j <= i + p - 1; ++j) {
                us[p + i] += ts[j] / p;
            }
        }

        /* Calculate basis function matrix */
        std::vector<Triplet> cA;
        cA.emplace_back(0, 0, 1.0);
        cA.emplace_back(n - 1, n - 1, 1.0);
        for (std::size_t i = 1; i < n - 1; ++i) {
            std::vector<T> bf(n, 0.0);

            T u = ts[i];
            auto it = std::upper_bound(us.begin(), us.end(), u);
            std::size_t k = std::distance(us.begin(), it) - 1;

            bf[k] = T(1.0);
            for (uint d = 1; d <= p; ++d) {
                bf[k - d] = (us[k + 1] - u) / (us[k + 1] - us[k - d + 1]) * bf[k - d + 1];
                for (uint j = k - d + 1; j < k; ++j) {
                    T l = (u - us[j]) / (us[j + d] - us[j]);
                    T r = (us[j + d + 1] - u) / (us[j + d + 1] - us[j + 1]);
                    bf[j] = l * bf[j] + r * bf[j + 1];
                }
                bf[k] = (u - us[k]) / (us[k + d] - us[k]) * bf[k];
            }

            for (std::size_t j = k - p; j <= k; ++j) {
                cA.emplace_back(i, j, bf[j]);
            }
        }

        SpMat A(n, n);
        A.setFromTriplets(cA.begin(), cA.end());

        points.resize(n);
        Eigen::MatrixXf B(n, N);
        Eigen::MatrixXf X(n, N);
        for (uint i = 0; i < n; ++i) {
            for (std::size_t j = 0; j < N; ++j) {
                B(i, j) = verts[i][j];
            }
        }

        Eigen::SparseQR<SpMat, Eigen::COLAMDOrdering<int>> solver;
        solver.compute(A);
        X = solver.solve(B);

        for (uint i = 0; i < n; ++i) {
            for (std::size_t j = 0; j < N; ++j) {
                points[i][j] = X(i, j);
            }
        }

        T error(0.0);
        for (uint i = 0; i < n; ++i) {
            error += (eval(ts[i]) - verts[i]).norm();
        }
        return error;
    }

    math::Vector<T, N> eval(T u) {
        if (u <= 0.0f) return points.front();
        if (u >= 1.0f) return points.back();
        auto it = std::upper_bound(us.begin(), us.end(), u);

        std::size_t k = std::distance(us.begin(), it) - 1;
        std::array<math::Vector<T, N>, p + 1> ps;
        for (uint i = 0; i <= p; ++i) {
            ps[i] = points[k - i];
        }
        for (uint i = 1; i <= p; ++i) {
            for (uint j = 0; j <= p - i; ++j) {
                T alpha = (u - us[k - j]) / (us[k - j + p + 1 - i] - us[k - j]);
                ps[j] = T(1.0 - alpha) * ps[j + 1] + alpha * ps[j];
            }
        }
        return ps[0];
    }
};

UTP_NAMESPACE_END

#endif /* UTP_BSPLINE_HEADER */
