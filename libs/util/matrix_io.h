#ifndef UTIL_MATRIX_IO_HEADER
#define UTIL_MATRIX_IO_HEADER

#include <fstream>

#include "math/matrix.h"

void
save_matrix_to_file(math::Matrix4f m, std::string const & filename) {
    std::ofstream out(filename.c_str());
    if (!out.good()) {
        throw std::runtime_error("Could not open matrix file");
    }

    out << m;

    out.close();
}

math::Matrix4f
load_matrix_from_file(std::string const & filename) {
    math::Matrix4f ret;
    std::ifstream in(filename.c_str());
    if (!in.good()) {
        throw std::runtime_error("Could not open matrix file");
    }

    for (int i = 0; i < 16; ++i) {
        in >> ret[i];
    }

    if (in.fail()) {
        in.close();
        throw std::runtime_error("Invalid matrix file");
    }

    in.close();

    return ret;
}

#endif /* UTIL_MATRIX_IO_HEADER */
