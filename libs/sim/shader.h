#ifndef SIM_SHADER_HEADER
#define SIM_SHADER_HEADER

#include "math/matrix.h"
#include "ogl/shader_program.h"

class Shader {
public:
    typedef std::shared_ptr<Shader> Ptr;

private:
    ogl::ShaderProgram::Ptr sp;
    bool modelview_dirty = true;
    bool proj_dirty = true;
    math::Matrix4f model;
    math::Matrix4f view;
    math::Matrix4f proj;

public:
    Shader() {
        math::matrix_set_identity(model.begin(), 4);
    }

    void load_shader_program(std::string const & path) {
        sp = ogl::ShaderProgram::create();
        if (!sp->try_load_all(path)) {
            throw std::runtime_error("Could not load shaders from: " + path);
        }
        update();
    }

    ogl::ShaderProgram::Ptr get_shader_program(void) {
        return sp;
    }

    void set_model_matrix(math::Matrix4f const & model_matrix) {
        modelview_dirty = true;
        model = model_matrix;
    }

    void set_view_matrix(math::Matrix4f const & view_matrix) {
        modelview_dirty = true;
        view = view_matrix;
    }

    void set_proj_matrix(math::Matrix4f const & proj_matrix) {
        proj_dirty = true;
        proj = proj_matrix;
    }

    void update(void) {
        if (proj_dirty || modelview_dirty) {
            sp->bind();
            if (proj_dirty) {
                sp->send_uniform("projmat", proj);
                proj_dirty = false;
            }
            if (modelview_dirty) {
                sp->send_uniform("modelviewmat", view * model);
                modelview_dirty = false;
            }
            sp->unbind();
        }
    }
};

#endif /* SIM_SHADER_HEADER */
