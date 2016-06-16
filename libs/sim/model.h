#ifndef SIM_MODEL_HEADER
#define SIM_MODEL_HEADER

#include <memory>
#include <vector>

#include "mve/mesh.h"
#include "ogl/texture.h"

#include "defines.h"

#include "shader_type.h"

struct Model {
    typedef std::shared_ptr<Model> Ptr;
    struct Part {
        mve::TriangleMesh::Ptr mesh;
        ogl::Texture::Ptr texture;
    };
    std::vector<Part> parts;
    ShaderType shader_type;
};

Model::Ptr
load_model(std::string const & path);

#endif /* SIM_MODEL_HEADER */
