#include <iostream>
#include <fstream>
#include <cerrno>
#include <cstring>
#include <array>
#include <mutex>
#include <condition_variable>

#include "util/arguments.h"
#include "util/file_system.h"
#include "util/system.h"
#include "util/exception.h"

#include "mve/mesh_io_ply.h"
#include "mve/mesh_io_obj.h"
#include "mve/image_io.h"
#include "mve/image_tools.h"

#include "ogl/mesh_renderer.h"
#include "ogl/render_tools.h"
#include "ogl/texture.h"
#include "ogl/camera.h"

#include "window.h"
#include "entity.h"

struct Arguments {
    std::string model;
};

Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_maxnum(1);
    args.set_nonopt_minnum(1);
    args.set_helptext_indent(28);
    args.set_usage("Usage: " + std::string(argv[0]) + " [OPTS] MODEL");
    args.set_description("TODO");
    args.parse(argc, argv);

    Arguments conf;
    conf.model = args.get_nth_nonopt(0);

    for (util::ArgResult const* i = args.next_option(); i != 0; i = args.next_option()) {
        switch (i->opt->sopt) {
        default:
            throw std::invalid_argument("Invalid option");
        }
    }
    return conf;
}

constexpr float pi = std::acos(-1.0f);

//TODO get rid of those global variables
std::array<ogl::ShaderProgram::Ptr, 4> shaders;
enum ShaderType {
    VCOLOR = 0,
    SURFACE = 1,
    TEXTURE = 2,
    LINES = 3
};
ogl::VertexArray::Ptr axes;
std::vector<Entity::Ptr> entities;
Pose::Ptr pose;

class ModelRenderer : public RenderComponent {
    public:
        typedef std::shared_ptr<ModelRenderer> Ptr;

    private:
        Pose::ConstPtr pose;

        struct Part {
            mve::TriangleMesh::Ptr mesh;
            ogl::MeshRenderer::Ptr mr;
            ogl::Texture::Ptr texture;
        };
        std::vector<Part> parts;
        ogl::ShaderProgram::Ptr shader;

    public:
        ModelRenderer(Pose::ConstPtr pose, ogl::ShaderProgram::Ptr shader)
            : pose(pose), shader(shader) {};

        void add_mesh(mve::TriangleMesh::Ptr mesh, ogl::Texture::Ptr texture)
        {
            parts.emplace_back();
            Part & part = parts.back();
            part.mesh = mesh;
            part.mr = ogl::MeshRenderer::create();
            part.mr->set_mesh(mesh);
            part.mr->set_shader(shader);
            part.texture = texture;
        }

        void render(void)
        {
            math::Matrix3f rot;
            pose->q.to_rotation_matrix(rot.begin());
            math::Vec3f trans = pose->x;
            math::Matrix4f m = rot.hstack(trans).vstack(math::Vec4f(0.0f, 0.0f, 0.0f, 1.0f));
            shader->bind();
            shader->send_uniform("modelmat", m);
            shader->unbind();
            for (Part const & part : parts) {
                if (part.texture != nullptr) {
                    part.texture->bind();
                }
                part.mr->draw();
            }
        }
};

class TrajectoryRenderer : public RenderComponent {
    private:
        Trajectory::ConstPtr trajectory;
        ogl::ShaderProgram::Ptr shader;

    public:
        TrajectoryRenderer(Trajectory::ConstPtr trajectory, ogl::ShaderProgram::Ptr shader)
            : trajectory(trajectory), shader(shader) {};

        void render(void)
        {
            mve::TriangleMesh::Ptr mesh(mve::TriangleMesh::create());
            mve::TriangleMesh::VertexList& verts(mesh->get_vertices());
            mve::TriangleMesh::FaceList& faces(mesh->get_faces());
            mve::TriangleMesh::ColorList& colors(mesh->get_vertex_colors());

            math::Vec4f color(0.0f, 0.0f, 1.0f, 1.0f);

            verts.insert(verts.end(), trajectory->xs.begin(), trajectory->xs.end());
            faces.resize(2 * (verts.size() - 1));
            colors.resize(verts.size(), color);

            for (uint i = 0; i < verts.size() - 1; ++i) {
                faces[2 * i + 0] = i;
                faces[2 * i + 1] = i + 1;
            }

            ogl::MeshRenderer::Ptr mr(ogl::MeshRenderer::create());
            mr->set_primitive(GL_LINES);
            mr->set_shader(shader);
            math::Matrix4f eye;
            math::matrix_set_identity(eye.begin(), 4);
            shader->bind();
            shader->send_uniform("modelmat", eye);
            shader->unbind();
            mr->set_mesh(mesh);
            mr->draw();
        }
};

class PropulsionRenderer : public RenderComponent {
    private:
        Pose::ConstPtr pose;
        Propulsion::ConstPtr propulsion;
        ogl::ShaderProgram::Ptr shader;

    public:
        PropulsionRenderer(Pose::ConstPtr pose, Propulsion::ConstPtr propulsion,
            ogl::ShaderProgram::Ptr shader)
            : pose(pose), propulsion(propulsion), shader(shader) {};

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

            for (int i = 0; i < num_engines; ++i) {
                verts[2 * i + 0] = pose->x + R * propulsion->rs[i];
                verts[2 * i + 1] = pose->x + R * propulsion->rs[i] + R.col(2) * (propulsion->thrusts[i] - 9.81f * 5.0f / 6.0f);
            }

            for (uint i = 0; i < 2 * num_engines; ++i) {
                faces[i] = i;
            }

            ogl::MeshRenderer::Ptr mr(ogl::MeshRenderer::create());
            mr->set_primitive(GL_LINES);
            mr->set_shader(shader);
            math::Matrix4f eye;
            math::matrix_set_identity(eye.begin(), 4);
            shader->bind();
            shader->send_uniform("modelmat", eye);
            shader->unbind();
            mr->set_mesh(mesh);
            mr->draw();
        }
};

struct Model {
    typedef std::shared_ptr<Model> Ptr;
    struct Part {
        mve::TriangleMesh::Ptr mesh;
        ogl::Texture::Ptr texture;
    };
    std::vector<Part> parts;
    ShaderType shader_type;
};


ogl::Texture::Ptr
load_texture(std::string const & path) {
    std::cout << "loading texture: " << path << std::endl;
    ogl::Texture::Ptr texture = ogl::Texture::create();
    mve::ByteImage::Ptr image;
    try {
        image = mve::image::load_file(path);
    } catch (std::exception& e) {
        std::cerr << "Could not load image: " << e.what() << std::endl;
        exit(EXIT_FAILURE);
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
            std::cerr << "\tCould not load mesh: "<< e.what() << std::endl;
            std::exit(EXIT_FAILURE);
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
            std::cerr << "Could not load model: " << e.what() << std::endl;
            exit(EXIT_FAILURE);
        }

        model->shader_type = TEXTURE;

        model->parts.resize(obj_model_parts.size());
        for (std::size_t i = 0; i < model->parts.size(); ++i) {
            model->parts[i].texture = load_texture(obj_model_parts[i].texture_filename);
            model->parts[i].mesh = obj_model_parts[i].mesh;
        }

    } else {
        std::cerr << "Unknown file extension " << ext << std::endl;
        exit(EXIT_FAILURE);
    }
    return model;
}

class Factory {
    public:
        Entity::Ptr static create_copter()
        {
            //TODO extract
            std::string basename = util::fs::dirname(util::fs::get_binary_path());
            mve::TriangleMesh::Ptr mesh;
            try {
                mesh = mve::geom::load_ply_mesh(basename + "/models/proxy.ply");
            } catch (std::exception& e) {
                std::cerr << "\tCould not load mesh: " << e.what() << std::endl;
                std::exit(EXIT_FAILURE);
            }

            pose = Pose::Ptr(new Pose);
            pose->x = math::Vec3f(0.0f, 0.0f, 20.0f);
            pose->q = math::Quatf(math::Vec3f(0.0f, 1.0f, 0.0f), 0.0f);
            Motion::Ptr motion(new Motion);
            motion->v = math::Vec3f(0.0f);
            motion->omega = math::Vec3f(0.0f);
            Physics::Ptr physics(new Physics);
            physics->mass = 5.0f;
            math::Matrix3f I(0.0f);
            I(0, 0) = 0.25 * physics->mass;
            I(1, 1) = 0.25 * physics->mass;
            I(2, 2) = 0.5 * physics->mass;
            physics->Ii = math::matrix_inverse(I);
            physics->drag_coeff = 0.75f;
            Propulsion::Ptr propulsion(new Propulsion);
            propulsion->rs.resize(6);
            propulsion->thrusts.resize(6);
            for (std::size_t i = 0; i < 6; ++i) {
                propulsion->thrusts[i] = (9.81f * physics->mass) / 6.0f;
                float theta = (2.0f * pi) / 6.0f * i + pi / 6.0f;
                propulsion->rs[i] = math::Vec3f(0.9 * std::cos(theta), 0.9 * std::sin(theta), 0.0f);
            }
            Trajectory::Ptr trajectory(new Trajectory);
            Control::Ptr control(new Control);
            control->omega = math::Vec3f(0.0f);

            PhysicsSimulator::Ptr ps(new PhysicsSimulator(pose, motion, physics, propulsion));
            Logger::Ptr l(new Logger(pose, motion, trajectory));
            Receiver::Ptr recv(new Receiver(control));
            FlightController::Ptr fc(new FlightController(pose, motion, control, propulsion));

            ModelRenderer::Ptr mr(new ModelRenderer(pose, shaders[VCOLOR]));
            mr->add_mesh(mesh, nullptr);
            TrajectoryRenderer::Ptr tr(new TrajectoryRenderer(trajectory, shaders[LINES]));
            PropulsionRenderer::Ptr pr(new PropulsionRenderer(pose, propulsion, shaders[LINES]));

            Entity::Ptr ret(new Entity);
            ret->add_component(pose);
            ret->add_component(motion);
            ret->add_component(physics);
            ret->add_component(propulsion);
            ret->add_component(trajectory);

            ret->add_component(ps);
            ret->add_component(l);
            ret->add_component(recv);
            ret->add_component(fc);

            ret->add_component(mr);
            ret->add_component(tr);
            ret->add_component(pr);
            return ret;
        }

        Entity::Ptr static create_model(std::string const & path)
        {
            Pose::Ptr pose(new Pose);
            pose->x = math::Vec3f(0.0f);
            pose->q = math::Quatf(math::Vec3f(0.0f, 0.0f, 1.0f), 0.0f);
            Entity::Ptr ret(new Entity);
            Model::Ptr model = load_model(path);
            ModelRenderer::Ptr mr(new ModelRenderer(pose, shaders[model->shader_type]));
            for (Model::Part const & part : model->parts) {
                mr->add_mesh(part.mesh, part.texture);
            }
            ret->add_component(pose);
            ret->add_component(mr);
            return ret;
        }
};

void
load_shader(ShaderType shader_type, std::string const & path) {
    shaders[shader_type] = ogl::ShaderProgram::create();
    if (!shaders[shader_type]->try_load_all(path)) {
        std::cerr << "\tCould not load shaders from: " << path << std::endl;
        std::exit(EXIT_FAILURE);
    }
}

void
load_shaders(void) {
    std::string basename = util::fs::dirname(util::fs::get_binary_path());
    load_shader(VCOLOR, basename + "/shaders/vcolor");
    load_shader(SURFACE, basename + "/shaders/surface");
    load_shader(TEXTURE, basename + "/shaders/texture");
    load_shader(LINES, basename + "/shaders/lines");
}

void
init_opengl(void) {
    glClearColor(0.9f, 0.9f, 0.9f, 1.0f);
    glEnable(GL_DEPTH_TEST);
}

void render(ogl::Camera const & cam) {
    for (ogl::ShaderProgram::Ptr & sp : shaders) {
        sp->bind();
        sp->send_uniform("viewmat", cam.view);
        sp->send_uniform("projmat", cam.proj);
        math::Matrix4f eye;
        math::matrix_set_identity(eye.begin(), 4);
        sp->send_uniform("modelmat", eye);
        sp->unbind();
    }
    axes->draw();
    for (Entity::Ptr const & entity : entities) {
        entity->render();
    }
}

void display(void) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    ogl::Camera camera;

    glViewport(0, 0, 960, 1080);
    camera.right = 0.05f;
    camera.top = 0.05f / (960.0f / 1080.0f);
    camera.pos = math::Vec3f(0.0f, 0.0f, 150.0f);
    camera.update_matrices();
    render(camera);

    math::Matrix3f rot;
    pose->q.to_rotation_matrix(rot.begin());
    math::Vec3f view_dir = rot.col(0);
    math::Vec3f right = rot.col(1);
    math::Vec3f up = rot.col(2);

    glViewport(960, 540, 960, 540);
    camera.top = 0.05f;
    camera.right = 0.05f * (960.0f / 540.0f);
    camera.pos = pose->x + view_dir * 0.85f;
    camera.viewing_dir = view_dir;
    camera.up_vec = up;
    camera.update_matrices();
    render(camera);

    glViewport(960, 0, 960, 540);
    camera.pos = pose->x + 1.5f * up - view_dir * 2.0f;
    camera.viewing_dir = math::matrix_rotation_from_axis_angle(right, pi / 4) * view_dir;
    camera.up_vec = up;
    camera.update_matrices();
    render(camera);

    glFlush();
}

void simulate(float delta) {
    for (Entity::Ptr const & entity : entities) {
        entity->update(delta);
    }
}

std::mutex m;
std::condition_variable cv;
bool pending = false;
bool done = false;
void save(mve::ByteImage::Ptr image) {
    uint num_images = 0;
    while (!done) {
        std::unique_lock<std::mutex> lk(m);
        cv.wait(lk);

        if (pending) {
            mve::image::flip<uint8_t>(image, mve::image::FLIP_VERTICAL);
            std::string filename = std::string("/tmp/test");
            filename += util::string::get_filled(num_images++, 4) + ".png";
            mve::image::save_png_file(image, filename);
            pending = false;
            std::cout << filename << std::endl;
        }
    }
}

/* ---------------------------------------------------------------- */

int main(int argc, char **argv) {
    util::system::register_segfault_handler();
    util::system::print_build_timestamp(argv[0]);

    Arguments args = parse_args(argc, argv);

    Window window("UAVMVS Simulator", 1920, 1080);
    init_opengl();

    load_shaders(); //TODO store shaders...

    axes = ogl::create_axis_renderer(shaders[LINES]);
    entities.push_back(Factory::create_copter());
    entities.push_back(Factory::create_model(args.model));

    if (glfwJoystickPresent(GLFW_JOYSTICK_1)) {
        std::cout << glfwGetJoystickName(GLFW_JOYSTICK_1) << std::endl;
    }

    int width = 960;
    int height = 540;
    mve::ByteImage::Ptr image = mve::ByteImage::create(width, height, 3);
    std::thread worker(save, image);

    double past = 0;
    double next = 1.0;
    while (window.good())
    {
        double now = glfwGetTime();
        float delta = now - past;
        past = now;

        if (now > next && !pending) {
            pending = true;
            glReadPixels(960, 540, width, height, GL_RGB, GL_UNSIGNED_BYTE, image->begin());
            std::lock_guard<std::mutex> lk(m);
            cv.notify_one();

            next += 1.0;
        }

        simulate(delta);
        display();

        window.update();

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    done = true;
    {
        std::lock_guard<std::mutex> lk(m);
        cv.notify_all();
    }
    worker.join();

    return EXIT_SUCCESS;
}
