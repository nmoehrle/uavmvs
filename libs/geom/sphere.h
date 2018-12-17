/*
 * Copyright (C) 2016-2018, Nils Moehrle
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#ifndef GEOM_SPHERE_HEADER
#define GEOM_SPHERE_HEADER

#include <map>
#include <unordered_map>

#include "mve/mesh.h"

mve::TriangleMesh::Ptr generate_sphere_mesh(float radius, uint subdivisions) {
    /* Derived from mve/apps/umve/scene_addins/addin_sphere_creator.cc */

    /* Initialize icosahedron */
    static std::vector<math::Vec3f> verts = {
        {0.0f, -0.5257311f, 0.8506508f},
        {0.0f, 0.5257311f, 0.8506508f},
        {0.0f, -0.5257311f, -0.8506508f},
        {0.0f, 0.5257311f, -0.8506508f},
        {0.8506508f, 0.0f, 0.5257311f},
        {0.8506508f, 0.0f, -0.5257311f},
        {-0.8506508f, 0.0f, 0.5257311f},
        {-0.8506508f, 0.0f, -0.5257311f},
        {0.5257311f, 0.8506508f, 0.0f},
        {0.5257311f, -0.8506508f, 0.0f},
        {-0.5257311f, 0.8506508f, 0.0f},
        {-0.5257311f, -0.8506508f, 0.0f}
    };

    static std::vector<uint> faces = {
        0, 4, 1,
        0, 9, 4,
        9, 5, 4,
        4, 5, 8,
        4, 8, 1,
        8, 10, 1,
        8, 3, 10,
        5, 3, 8,
        5, 2, 3,
        2, 7, 3,
        7, 10, 3,
        7, 6, 10,
        7, 11, 6,
        11, 0, 6,
        0, 1, 6,
        6, 1, 10,
        9, 0, 11,
        9, 11, 2,
        9, 2, 5,
        7, 2, 11,
    };

    /* Icosahedron subdivision. */
    for (uint i = 0; i < subdivisions; ++i) {
        /* Walk over faces and generate vertices. */
        typedef std::pair<uint, uint> Edge;
        typedef std::pair<Edge, uint> MapValueType;
        std::map<Edge, uint> edge_map;
        for (std::size_t j = 0; j < faces.size(); j += 3)
        {
            uint v0 = faces[j + 0];
            uint v1 = faces[j + 1];
            uint v2 = faces[j + 2];
            edge_map.insert(MapValueType(Edge(v0, v1), verts.size()));
            edge_map.insert(MapValueType(Edge(v1, v0), verts.size()));
            verts.push_back((verts[v0] + verts[v1]) / 2.0f);
            edge_map.insert(MapValueType(Edge(v1, v2), verts.size()));
            edge_map.insert(MapValueType(Edge(v2, v1), verts.size()));
            verts.push_back((verts[v1] + verts[v2]) / 2.0f);
            edge_map.insert(MapValueType(Edge(v2, v0), verts.size()));
            edge_map.insert(MapValueType(Edge(v0, v2), verts.size()));
            verts.push_back((verts[v2] + verts[v0]) / 2.0f);
        }

        /* Walk over faces and create new connectivity. */
        std::size_t num_faces = faces.size();
        for (std::size_t j = 0; j < num_faces; j += 3) {
            uint v0 = faces[j + 0];
            uint v1 = faces[j + 1];
            uint v2 = faces[j + 2];
            uint ev0 = edge_map[Edge(v0, v1)];
            uint ev1 = edge_map[Edge(v1, v2)];
            uint ev2 = edge_map[Edge(v2, v0)];
            faces.push_back(ev0); faces.push_back(v1); faces.push_back(ev1);
            faces.push_back(ev1); faces.push_back(v2); faces.push_back(ev2);
            faces.push_back(ev2); faces.push_back(v0); faces.push_back(ev0);
            faces[j + 0] = ev0;
            faces[j + 1] = ev1;
            faces[j + 2] = ev2;
        }
    }

    mve::TriangleMesh::Ptr mesh = mve::TriangleMesh::create();
    std::vector<math::Vec3f> & normals = mesh->get_vertex_normals();
    normals.resize(verts.size());

    /* Normalize and transform vertices. */
    for (std::size_t i = 0; i < verts.size(); ++i) {
        normals[i] = verts[i].normalized();
        verts[i] = normals[i] * radius;
    }

    mesh->get_vertices().swap(verts);
    mesh->get_faces().swap(faces);

    mesh->recalc_normals(true, false);

    return mesh;
}

void parameterize_spherical(mve::TriangleMesh::Ptr sphere) {
    std::vector<uint> & faces = sphere->get_faces();
    std::vector<math::Vec3f> & verts = sphere->get_vertices();
    std::vector<math::Vec3f> & normals = sphere->get_vertex_normals();
    std::vector<math::Vec2f> & texcoords = sphere->get_vertex_texcoords();
    std::vector<float> * confidences = nullptr;
    if (sphere->has_vertex_confidences()) {
        *confidences = sphere->get_vertex_confidences();
    }
    std::vector<float> * values = nullptr;
    if (sphere->has_vertex_values()) {
        *values = sphere->get_vertex_values();
    }
    std::vector<math::Vec4f> * colors = nullptr;
    if (sphere->has_vertex_colors()) {
        *colors = sphere->get_vertex_colors();
    }
    texcoords.resize(verts.size());

    constexpr float pi = 3.14159265f;

    for (std::size_t i = 0; i < verts.size(); ++i) {
        math::Vec3f const & vert = verts[i];
        float theta = std::acos(vert[2] / vert.norm());
        float phi = std::atan2(vert[1], vert[0]);
        texcoords[i] = math::Vec2f(phi / (2 * pi), theta / pi);
    }

    uint num_faces = faces.size() / 3;

    std::unordered_map<uint, uint> new_vert_ids;

    /* Find and split vertices on seam. */
    for (std::size_t i = 0; i < num_faces * 3; i += 3) {
        uint vid0 = faces[i + 0];
        uint vid1 = faces[i + 1];
        uint vid2 = faces[i + 2];
        math::Vec2f const & uv0 = texcoords[vid0];
        math::Vec2f const & uv1 = texcoords[vid1];
        math::Vec2f const & uv2 = texcoords[vid2];

        float l01 = (uv1 - uv0).norm();
        float l12 = (uv2 - uv1).norm();
        float l20 = (uv0 - uv2).norm();

        uint * vid = nullptr;

        if (l12 < 0.5f && l20 > 0.5f && l01 > 0.5f)  {
            vid = &faces[i + 0];
        }
        if (l20 < 0.5f && l01 > 0.5f && l12 > 0.5f) {
            vid = &faces[i + 1];
        }
        if (l01 < 0.5f && l12 > 0.5f && l20 > 0.5f) {
            vid = &faces[i + 2];
        }

        if (vid != nullptr) {
            auto it = new_vert_ids.find(*vid);
            if (it != new_vert_ids.end()) {
                *vid = it->second;
            } else {
                uint new_vert_id = new_vert_ids[*vid] = verts.size();
                verts.push_back(verts[*vid]);
                normals.push_back(normals[*vid]);
                if (colors) colors->push_back(colors->at(*vid));
                if (confidences) confidences->push_back(confidences->at(*vid));
                if (values) values->push_back(values->at(*vid));
                math::Vec2f new_uv = texcoords[*vid];
                new_uv[0] += (new_uv[0] < 0.5f) ? 1.0f : -1.0f;
                texcoords.push_back(new_uv);
                *vid = new_vert_id;
            }
        }
    }
    sphere->recalc_normals(true, false);
}

#endif /* GEOM_SPHERE_HEADER */
