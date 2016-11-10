#ifndef GEOM_CLOUD_IO_HEADER
#define GEOM_CLOUD_IO_HEADER

#include <iostream>
#include <fstream>
#include <cstring>

#include "mve/mesh.h"
#include "math/matrix.h"

mve::TriangleMesh::Ptr load_ptx_cloud(const std::string & filename) {
    std::ifstream in(filename.c_str(), std::ios::binary);
    if (!in.good()) {
        throw util::FileException(filename, std::strerror(errno));
    }

    mve::TriangleMesh::Ptr ret = mve::TriangleMesh::create();
    std::vector<math::Vec3f> & verts = ret->get_vertices();
    std::vector<math::Vec4f> & colors = ret->get_vertex_colors();
    std::vector<float> & values = ret->get_vertex_values();

    std::string buffer;

    std::size_t idx = 0;
    std::size_t cols, rows;
    while (in >> cols >> rows) {
        /* Discard the rest of the line. */
        std::getline(in, buffer);

        /* Ignore scanner position and axis. */
        for (int i = 0; i < 4; ++i) std::getline(in, buffer);

        math::Matrix4f T;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                in >> T(j, i);
            }
        }
        std::getline(in, buffer);

        verts.resize(idx + rows * cols);
        values.resize(idx + rows * cols);
        colors.resize(idx + rows * cols);

        for (;idx < verts.size(); ++idx) {
            math::Vec3f pos;
            in >> pos[0] >> pos[1] >> pos[2];
            verts[idx] = T.mult(pos, 1.0);

            in >> values[idx];

            math::Vec4f c;
            in >> c[0] >> c[1] >> c[2];
            c[3] = 255.0f;
            colors[idx] = c / 255.0f;
        }

        /* Discard the rest of the line. */
        std::getline(in, buffer);

        if (in.fail()) {
            in.close();
            throw util::FileException(filename, "Error reading ptx file");
        }
    }

    return ret;
}

#endif /* GEOM_CLOUD_IO_HEADER */
