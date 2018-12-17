/*
 * Copyright (C) 2016-2018, Nils Moehrle
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#ifndef UTIL_MATRIX_IO_HEADER
#define UTIL_MATRIX_IO_HEADER

#include <fstream>

#include "math/matrix.h"

template <typename T, int N, int M>
void
save_matrix_to_file(typename math::Matrix<T, N, M> m, std::string const & filename) {
    std::ofstream out(filename.c_str());
    if (!out.good()) {
        throw std::runtime_error("Could not open matrix file");
    }

    out << std::fixed << m;

    out.close();
}

template <typename T, int N, int M>
typename math::Matrix<T, N, M>
load_matrix_from_file(std::string const & filename) {
    typename math::Matrix<T, N, M> ret;
    std::ifstream in(filename.c_str());
    if (!in.good()) {
        throw std::runtime_error("Could not open matrix file");
    }

    for (int i = 0; i < N * M; ++i) {
        in >> ret[i];
    }

    if (in.fail()) {
        in.close();
        throw std::runtime_error("Invalid matrix file or dimension");
    }

    in.close();

    return ret;
}

#endif /* UTIL_MATRIX_IO_HEADER */
