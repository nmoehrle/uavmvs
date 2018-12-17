/*
 * Copyright (C) 2016-2018, Nils Moehrle
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#ifndef GEOM_TRANSFORM_HEADER
#define GEOM_TRANSFORM_HEADER

#include <tuple>

#include "math/matrix_tools.h"

template <typename T>
std::tuple<typename math::Matrix<T, 3, 3>, T, typename math::Vector<T, 3> >
split_transform(typename math::Matrix<T, 4, 4> const & Tr) {
    typename math::Matrix<T, 3, 3> R;
    R(0, 0) = Tr(0, 0); R(0, 1) = Tr(0, 1); R(0, 2) = Tr(0, 2);
    R(1, 0) = Tr(1, 0); R(1, 1) = Tr(1, 1); R(1, 2) = Tr(1, 2);
    R(2, 0) = Tr(2, 0); R(2, 1) = Tr(2, 1); R(2, 2) = Tr(2, 2);
    T s = std::cbrt(math::matrix_determinant(R));
    R = R / s;
    typename math::Vector<T, 3> t(Tr(0, 3), Tr(1, 3), Tr(2, 3));
    return std::make_tuple(R, s, t);
}

template <typename T>
typename math::Matrix<T, 4, 4>
assemble_transform(typename math::Matrix<T, 3, 3> R, T s, typename math::Vector<T, 3> t) {
    return (R * s).hstack(t).vstack(math::Vector<T, 4>(0.0f, 0.0f, 0.0f, 1.0f));
}

template <typename T>
typename math::Matrix<T, 3, 3>
extract_rotation(typename math::Matrix<T, 4, 4> const & Tr) {
    typename math::Matrix<T, 3, 3> R;
    std::tie(R, std::ignore, std::ignore) = split_transform(Tr);
    return R;
}

#endif /* GEOM_TRANSFORM_HEADER */
