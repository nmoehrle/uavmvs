#ifndef ENTITY_TRAJECTORY_RENDERER_HEADER
#define ENTITY_TRAJECTORY_RENDERER_HEADER

#include "sim/entity.h"
#include "sim/shader.h"

class TrajectoryRenderer : public RenderComponent {
    private:
        Trajectory::ConstPtr trajectory;
        Shader::Ptr shader;

    public:
        TrajectoryRenderer(Trajectory::ConstPtr trajectory, Shader::Ptr shader)
            : trajectory(trajectory), shader(shader) {};

        void render(void)
        {
            mve::TriangleMesh::Ptr mesh(mve::TriangleMesh::create());
            mve::TriangleMesh::VertexList& verts(mesh->get_vertices());
            mve::TriangleMesh::FaceList& faces(mesh->get_faces());
            mve::TriangleMesh::ColorList& colors(mesh->get_vertex_colors());

            math::Vec4f color(0.0f, 0.0f, 1.0f, 1.0f);

            verts.insert(verts.end(), trajectory->xs.begin(), trajectory->xs.end());
            if (verts.empty()) return;
            faces.resize(2 * (verts.size() - 1));
            colors.resize(verts.size(), color);

            for (uint i = 0; i < verts.size() - 1; ++i) {
                faces[2 * i + 0] = i;
                faces[2 * i + 1] = i + 1;
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

#endif /* ENTITY_TRAJECTORY_RENDERER_HEADER */
