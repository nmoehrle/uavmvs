/*
 * Copyright (C) 2016-2018, Nils Moehrle
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#ifndef GEOM_AABB_HEADER
#define GEOM_AABB_HEADER

#include "acc/primitives.h"

#include "mve/mesh.h"

acc::AABB<math::Vec3f>
estimate_aabb(std::vector<math::Vec3f> verts, float epsilon) {
    acc::AABB<math::Vec3f> aabb;

    for (int i = 0; i < 3; ++i) {
        std::vector<math::Vec3f>::iterator nth;
        nth = verts.begin() + verts.size() * epsilon;
        std::nth_element(verts.begin(), nth, verts.end(),
            [i] (math::Vec3f const & l, math::Vec3f const & r) -> bool {
                return l[i] < r[i];
            }
        );
        aabb.min[i] = (*nth)[i];

        nth = verts.begin() + verts.size() * epsilon;
        std::nth_element(verts.begin(), nth, verts.end(),
            [i] (math::Vec3f const & l, math::Vec3f const & r) -> bool {
                return l[i] > r[i];
            }
        );
        aabb.max[i] = (*nth)[i];
    }

    return aabb;
}

mve::TriangleMesh::Ptr
generate_aabb_mesh(math::Vec3f const & aabb_min, math::Vec3f const & aabb_max) {
    /* Derived from mve/apps/umve/scene_addins/addin_aabb_creator.cc */

    mve::TriangleMesh::Ptr mesh = mve::TriangleMesh::create();
    mve::TriangleMesh::VertexList& verts = mesh->get_vertices();
    mve::TriangleMesh::FaceList& faces = mesh->get_faces();
    mve::TriangleMesh::ColorList& colors = mesh->get_vertex_colors();
    verts.resize(6 * 4);
    faces.reserve(6 * 2 * 3);
    colors.resize(6 * 4, math::Vec4f(0.7f, 0.7f, 0.7f, 0.5f));

    /* Bottom vertices. */
    verts[0]  = math::Vec3f(aabb_min[0], aabb_min[1], aabb_min[2]);
    verts[1]  = math::Vec3f(aabb_max[0], aabb_min[1], aabb_min[2]);
    verts[2]  = math::Vec3f(aabb_min[0], aabb_min[1], aabb_max[2]);
    verts[3]  = math::Vec3f(aabb_max[0], aabb_min[1], aabb_max[2]);
    /* Top vertices. */
    verts[4]  = math::Vec3f(aabb_min[0], aabb_max[1], aabb_min[2]);
    verts[5]  = math::Vec3f(aabb_max[0], aabb_max[1], aabb_min[2]);
    verts[6]  = math::Vec3f(aabb_min[0], aabb_max[1], aabb_max[2]);
    verts[7]  = math::Vec3f(aabb_max[0], aabb_max[1], aabb_max[2]);
    /* Back vertices. */
    verts[8]  = math::Vec3f(aabb_min[0], aabb_min[1], aabb_min[2]);
    verts[9]  = math::Vec3f(aabb_max[0], aabb_min[1], aabb_min[2]);
    verts[10] = math::Vec3f(aabb_min[0], aabb_max[1], aabb_min[2]);
    verts[11] = math::Vec3f(aabb_max[0], aabb_max[1], aabb_min[2]);
    /* Front vertices. */
    verts[12] = math::Vec3f(aabb_min[0], aabb_min[1], aabb_max[2]);
    verts[13] = math::Vec3f(aabb_max[0], aabb_min[1], aabb_max[2]);
    verts[14] = math::Vec3f(aabb_min[0], aabb_max[1], aabb_max[2]);
    verts[15] = math::Vec3f(aabb_max[0], aabb_max[1], aabb_max[2]);
    /* Left vertices. */
    verts[16] = math::Vec3f(aabb_min[0], aabb_min[1], aabb_min[2]);
    verts[17] = math::Vec3f(aabb_min[0], aabb_max[1], aabb_min[2]);
    verts[18] = math::Vec3f(aabb_min[0], aabb_min[1], aabb_max[2]);
    verts[19] = math::Vec3f(aabb_min[0], aabb_max[1], aabb_max[2]);
    /* Right vertices. */
    verts[20] = math::Vec3f(aabb_max[0], aabb_min[1], aabb_min[2]);
    verts[21] = math::Vec3f(aabb_max[0], aabb_max[1], aabb_min[2]);
    verts[22] = math::Vec3f(aabb_max[0], aabb_min[1], aabb_max[2]);
    verts[23] = math::Vec3f(aabb_max[0], aabb_max[1], aabb_max[2]);

    /* Bottom faces. */
    faces.push_back(0); faces.push_back(1); faces.push_back(2);
    faces.push_back(1); faces.push_back(3); faces.push_back(2);
    /* Top faces. */
    faces.push_back(4); faces.push_back(7); faces.push_back(5);
    faces.push_back(4); faces.push_back(6); faces.push_back(7);
    /* Back faces. */
    faces.push_back(8); faces.push_back(10); faces.push_back(9);
    faces.push_back(9); faces.push_back(10); faces.push_back(11);
    /* Front faces. */
    faces.push_back(12); faces.push_back(13); faces.push_back(14);
    faces.push_back(13); faces.push_back(15); faces.push_back(14);
    /* Left faces. */
    faces.push_back(16); faces.push_back(18); faces.push_back(17);
    faces.push_back(17); faces.push_back(18); faces.push_back(19);
    /* Right faces. */
    faces.push_back(20); faces.push_back(21); faces.push_back(22);
    faces.push_back(21); faces.push_back(23); faces.push_back(22);

    mesh->recalc_normals();
    return mesh;
}

#endif /* GEOM_AABB_HEADER */
