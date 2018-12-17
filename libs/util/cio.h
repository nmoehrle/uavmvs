/*
 * Copyright (C) 2016-2018, Nils Moehrle
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include <iostream>

#include "mve/mesh_io_ply.h"
#include "cacc/point_cloud.h"

cacc::PointCloud<cacc::HOST>::Ptr
load_point_cloud(std::string const & path)
{
    mve::TriangleMesh::Ptr mesh;
    try {
        mesh = mve::geom::load_ply_mesh(path);
    } catch (std::exception& e) {
        std::cerr << "\tCould not load point cloud: " << e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }

    if (!mesh->has_vertex_normals() && mesh->get_faces().size() != 0) {
        mesh->recalc_normals(false, true);
    }

    if (!mesh->has_vertex_normals()) {
        std::cerr << "\tPoint cloud has no vertex normals." << std::endl;
        std::exit(EXIT_FAILURE);
    }

    std::vector<math::Vec3f> const & vertices = mesh->get_vertices();
    std::vector<math::Vec3f> const & normals = mesh->get_vertex_normals();
    std::vector<float> const & values = mesh->get_vertex_values();
    std::vector<float> const & confidences = mesh->get_vertex_confidences();

    cacc::PointCloud<cacc::HOST>::Ptr ret;
    ret = cacc::PointCloud<cacc::HOST>::create(vertices.size());
    cacc::PointCloud<cacc::HOST>::Data data = ret->cdata();
    for (std::size_t i = 0; i < vertices.size(); ++i) {
        data.vertices_ptr[i] = cacc::Vec3f(vertices[i].begin());
        data.normals_ptr[i] = cacc::Vec3f(normals[i].begin());
        data.values_ptr[i] = values.size() ? values[i] : 0.0f;
        data.qualities_ptr[i] = confidences.size() ? confidences[i] : 1.0f;
    }

    return ret;
}

