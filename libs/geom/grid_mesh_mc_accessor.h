/*
 * Copyright (C) 2016-2018, Nils Moehrle
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#ifndef GRID_MESH_MC_ACCESSOR_HEADER
#define GRID_MESH_MC_ACCESSOR_HEADER

/* Marching cubes accessor c.f. mve/libs/mve/volume.cc */
class GridMeshMCAccessor {
private:
    mve::TriangleMesh::ConstPtr mesh;
    int iter;
    int width;
    int height;
    int depth;

public:
    float sdf[8];
    std::size_t vid[8];
    math::Vec3f pos[8];
    math::Vec3f color[8];

public:
    GridMeshMCAccessor(mve::TriangleMesh::Ptr mesh, int width, int height, int depth)
        : mesh(mesh), iter(-1), width(width), height(height), depth(depth) {};

    bool next (void) {
        this->iter += 1;
        if (this->iter == (width - 1) * (height - 1) * (depth - 1)) {
            return false;
        }

        int const base_x = iter % (width - 1);
        int const base_y = (iter / (width - 1)) % (height - 1);
        int const base_z = iter / ((width - 1) * (height - 1));
        int const base = base_z * width * height + base_y * width + base_x;

        this->vid[0] = base;
        this->vid[1] = base + 1;
        this->vid[2] = base + 1 + width * height;
        this->vid[3] = base + width * height;
        this->vid[4] = base + width;
        this->vid[5] = base + 1 + width;
        this->vid[6] = base + 1 + width + width * height;
        this->vid[7] = base + width + width * height;

        std::vector<math::Vec3f> const & verts = this->mesh->get_vertices();
        std::vector<float> const & values = this->mesh->get_vertex_values();
        std::vector<math::Vec4f> const & colors = this->mesh->get_vertex_colors();

        for (int i = 0; i < 8; ++i) {
            std::size_t vid = this->vid[i];
            this->sdf[i] = values[vid];
            this->pos[i] = verts[vid];
            if (mesh->has_vertex_colors()) {
                float const * color = colors[vid].begin();
                std::copy(color, color + 3, this->color[i].begin());
            }
        }

        return true;
    }

    bool has_colors (void) const {
        return this->mesh->has_vertex_colors();
    }
};

#endif /* GRID_MESH_MC_ACCESSOR_HEADER */
