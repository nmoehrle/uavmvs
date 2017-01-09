#ifndef GEOM_AABB_IO_HEADER
#define GEOM_AABB_IO_HEADER

#include <fstream>

#include "math/vector.h"

#include "acc/primitives.h"

void
save_aabb_to_file(acc::AABB<math::Vec3f> aabb, std::string const & filename) {
    std::ofstream out(filename.c_str());
    if (!out.good()) {
        throw std::runtime_error("Could not open AABB file");
    }

    out << aabb.min << ' ' << aabb.max;

    out.close();
}

acc::AABB<math::Vec3f>
load_aabb_from_file(std::string const & filename) {
    acc::AABB<math::Vec3f> aabb;
    std::ifstream in(filename.c_str());
    if (!in.good()) {
        throw std::runtime_error("Could not open AABB file");
    }

    for (int i = 0; i < 3; ++i) {
        in >> aabb.min[i];
    }
    for (int i = 0; i < 3; ++i) {
        in >> aabb.max[i];
    }

    if (in.fail()) {
        in.close();
        throw std::runtime_error("Invalid AABB file");
    }

    in.close();

    return aabb;
}

#endif /* GEOM_AABB_IO_HEADER */
