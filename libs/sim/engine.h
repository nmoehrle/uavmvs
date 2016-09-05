#include <unordered_map>

#include "util/file_system.h"

#include "model.h"
#include "entity.h"
#include "shader.h"
#include "shader_type.h"
#include "model_renderer.h"

class Engine {
public:
    typedef std::shared_ptr<Engine> Ptr;

private:
    std::string basepath;
    std::vector<Entity::Ptr> entities;
    std::unordered_map<ShaderType, Shader::Ptr, std::hash<char> > shaders;

protected:
    Shader::Ptr get_shader(ShaderType shader_type) {
        auto it = shaders.find(shader_type);
        if (it != shaders.end()) return it->second;

        Shader::Ptr ret(new Shader());
        std::string path = util::fs::join_path(__ROOT__, "res/shaders");
        ret->load_shader_program(path + "/" + shader_names[shader_type]);
        shaders[shader_type] = ret;
        return ret;
    }

public:
    void update(double delta_time) {
        for (Entity::Ptr const & entity : entities) {
            entity->update(delta_time);
        }
    }

    void render(ogl::Camera const & cam) {
        for (auto const & elem : shaders) {
            Shader::Ptr const & shader = elem.second;
            shader->set_view_matrix(cam.view);
            shader->set_proj_matrix(cam.proj);
            shader->update();
        }

        for (Entity::Ptr const & entity : entities) {
            entity->render();
        }
    }

    void add_entity(Entity::Ptr entity) {
        entities.push_back(entity);
    }

    Entity::Ptr create_static_model(std::string const & path)
    {
        Entity::Ptr ret(new Entity);
        Model::Ptr model = load_model(path);
        Shader::Ptr shader = this->get_shader(model->shader_type);
        ModelRenderer::Ptr mr(new ModelRenderer(shader));
        for (Model::Part const & part : model->parts) {
            mr->add_mesh(part.mesh, part.texture);
        }
        ret->add_component(mr);
        entities.push_back(ret);
        return ret;
    }
};
