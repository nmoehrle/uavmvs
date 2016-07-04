#include "mve/image_io.h"
#include "mve/mesh_io_ply.h"
#include "mve/mesh_io_obj.h"

#include "ogl/texture.h"
#include "ogl/check_gl_error.h"

#include "shader_type.h"
#include "model.h" //IO???

ogl::Texture::Ptr
load_texture(std::string const & path) {
    ogl::Texture::Ptr texture = ogl::Texture::create();
    mve::ByteImage::Ptr image;
    try {
        image = mve::image::load_file(path);
    } catch (std::exception& e) {
        throw std::runtime_error(std::string("Could not load image: ") + e.what());
    }
    texture->upload(image);

    texture->bind();
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    ogl::check_gl_error();

    return texture;
}

Model::Ptr
load_model(std::string const & path) {
    std::string ext = util::string::right(path, 4);

    Model::Ptr model = Model::Ptr(new Model());
    if (ext == ".ply") {
        mve::TriangleMesh::Ptr mesh;
        try {
            mesh = mve::geom::load_ply_mesh(path);
        } catch (std::exception& e) {
            throw std::runtime_error(std::string("Could not load mesh: ") + e.what());
        }
        mesh->ensure_normals(true, true);

        model->shader_type = mesh->has_vertex_colors() ? VCOLOR : SURFACE;

        model->parts.emplace_back();
        Model::Part & part = model->parts.back();
        part.mesh = mesh;
        part.texture = nullptr;
    } else if (ext == ".obj") {
        std::vector<mve::geom::ObjModelPart> obj_model_parts;
        try {
             mve::geom::load_obj_mesh(path, &obj_model_parts);
        } catch (std::exception& e) {
            throw std::runtime_error(std::string("Could not load model: ") + e.what());
        }

        model->shader_type = TEXTURE;

        model->parts.resize(obj_model_parts.size());
        for (std::size_t i = 0; i < model->parts.size(); ++i) {
            model->parts[i].texture = load_texture(obj_model_parts[i].texture_filename);
            model->parts[i].mesh = obj_model_parts[i].mesh;
        }

    } else {
        throw std::runtime_error(std::string("Unknown file extension ") + ext);
    }
    return model;
}
