#ifndef GEOM_SPHERE_HEADER
#define GEOM_SPHERE_HEADER

mve::TriangleMesh::Ptr generate_sphere(float radius, uint subdivisions) {
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

    /* Normalize and transform vertices. */
    for (std::size_t i = 0; i < verts.size(); ++i) {
        verts[i].normalize();
        verts[i] = verts[i] * radius;
    }

    mve::TriangleMesh::Ptr mesh = mve::TriangleMesh::create();
    mesh->get_vertices().swap(verts);
    mesh->get_faces().swap(faces);

    mesh->recalc_normals();
    return mesh;
}

#endif /* GEOM_SPHERE_HEADER */
