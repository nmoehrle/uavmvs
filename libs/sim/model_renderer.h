#ifndef SIM_MODELRENDERER_HEADER
#define SIM_MODELRENDERER_HEADER

#include "ogl/texture.h"
#include "ogl/mesh_renderer.h"

#include "defines.h"

#include "shader.h"

class ModelRenderer : public RenderComponent {
public:
    typedef std::shared_ptr<ModelRenderer> Ptr;

private:
    struct Part {
        mve::TriangleMesh::Ptr mesh;
        ogl::MeshRenderer::Ptr mr;
        ogl::Texture::Ptr texture;
    };
    std::vector<Part> parts;

protected:
    Shader::Ptr shader;

public:
    ModelRenderer(Shader::Ptr shader)
        : shader(shader) {};

    void add_mesh(mve::TriangleMesh::Ptr mesh, ogl::Texture::Ptr texture)
    {
        parts.emplace_back();
        Part & part = parts.back();
        part.mesh = mesh;
        part.mr = ogl::MeshRenderer::create();
        part.mr->set_mesh(mesh);
        part.mr->set_shader(shader->get_shader_program());
        part.texture = texture;
    }

    void render(void)
    {
        for (Part const & part : parts) {
            if (part.texture != nullptr) {
                part.texture->bind();
            }
            part.mr->draw();
        }
    }
};

class DynamicModelRenderer : public ModelRenderer {
private:
    Pose::ConstPtr pose;

public:
    DynamicModelRenderer(Pose::ConstPtr pose, Shader::Ptr shader)
        : ModelRenderer(shader), pose(pose) {};

    void render(void)
    {
        math::Matrix3f rot;
        pose->q.to_rotation_matrix(rot.begin());
        math::Vec3f trans = pose->x;
        math::Matrix4f m = rot.hstack(trans).vstack(math::Vec4f(0.0f, 0.0f, 0.0f, 1.0f));
        shader->set_model_matrix(m);
        shader->update();

        ModelRenderer::render();
    }
};

#endif /* SIM_MODELRENDERER_HEADER */
