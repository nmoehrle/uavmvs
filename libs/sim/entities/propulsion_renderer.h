/*
 * Copyright (C) 2015, Nils Moehrle
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#ifndef ENTITY_PROPULSION_RENDERER_HEADER
#define ENTITY_PROPULSION_RENDERER_HEADER

#include "sim/entity.h"
#include "sim/shader.h"

class PropulsionRenderer : public RenderComponent {
    private:
        Pose::ConstPtr pose;
        Physics::ConstPtr physics;
        Propulsion::ConstPtr propulsion;
        Shader::Ptr shader;

    public:
        PropulsionRenderer(Pose::ConstPtr pose, Physics::ConstPtr physics,
            Propulsion::ConstPtr propulsion, Shader::Ptr shader)
            : pose(pose), physics(physics), propulsion(propulsion), shader(shader) {};

        void render(void)
        {
            mve::TriangleMesh::Ptr mesh(mve::TriangleMesh::create());
            mve::TriangleMesh::VertexList& verts(mesh->get_vertices());
            mve::TriangleMesh::FaceList& faces(mesh->get_faces());
            mve::TriangleMesh::ColorList& colors(mesh->get_vertex_colors());

            uint num_engines = 6;//propulsion->engines.size();

            math::Vec4f color(0.0f, 1.0f, 0.0f, 1.0f);

            verts.resize(2 * num_engines);
            faces.resize(2 * num_engines);
            colors.resize(2 * num_engines, color);

            math::Matrix3f R;
            pose->q.to_rotation_matrix(R.begin());

            for (uint i = 0; i < num_engines; ++i) {
                verts[2 * i + 0] = pose->x + R * propulsion->rs[i];
                float rel_thrust = propulsion->thrusts[i] - 9.81f * physics->mass / num_engines;
                verts[2 * i + 1] = pose->x + R * propulsion->rs[i] + R.col(2) * rel_thrust * 0.1f;
            }

            for (uint i = 0; i < 2 * num_engines; ++i) {
                faces[i] = i;
            }

            ogl::MeshRenderer::Ptr mr(ogl::MeshRenderer::create());
            mr->set_primitive(GL_LINES);
            mr->set_shader(shader->get_shader_program());
            math::Matrix4f eye;
            math::matrix_set_identity(eye.begin(), 4);
            shader->set_model_matrix(eye);
            shader->update();
            mr->set_mesh(mesh);
            mr->draw();
        }
};

#endif /* ENTITY_PROPULSION_RENDERER_HEADER */
