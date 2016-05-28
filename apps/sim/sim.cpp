#include <iostream>
#include <fstream>
#include <cerrno>
#include <cstring>
#include <array>

#include "util/arguments.h"
#include "util/file_system.h"
#include "util/system.h"
#include "util/exception.h"

#include "math/quaternion.h"

#include "mve/mesh_io_ply.h"
#include "mve/mesh_io_obj.h"
#include "mve/image_io.h"

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

std::array<ogl::ShaderProgram::Ptr, 3> shaders;
enum ShaderType {
    VCOLOR = 0,
    SURFACE = 1,
    TEXTURE = 2
};

class MeshRenderer : public RenderComponent {
    public:
        typedef std::shared_ptr<MeshRenderer> Ptr;

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
        MeshRenderer(Pose::ConstPtr pose)
            : pose(pose) {};
        void set_shader(ogl::ShaderProgram::Ptr shader)
        {
            this->shader = shader;
        }

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

    public:
        TrajectoryRenderer(Trajectory::ConstPtr trajectory)
            : trajectory(trajectory) {};

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
            mr->set_shader(shaders[VCOLOR]);
            mr->set_mesh(mesh);
            mr->draw();
        }
};

class PropulsionRenderer : public RenderComponent {
    private:
        Pose::ConstPtr pose;
        Propulsion::ConstPtr propulsion;

    public:
        PropulsionRenderer(Pose::ConstPtr pose, Propulsion::ConstPtr propulsion)
            : pose(pose), propulsion(propulsion) {};

        void render(void)
        {
            //TODO support n
            mve::TriangleMesh::Ptr mesh(mve::TriangleMesh::create());
            mve::TriangleMesh::VertexList& verts(mesh->get_vertices());
            mve::TriangleMesh::FaceList& faces(mesh->get_faces());
            mve::TriangleMesh::ColorList& colors(mesh->get_vertex_colors());

            math::Vec4f color(0.0f, 1.0f, 0.0f, 1.0f);

            verts.resize(12);
            faces.resize(12);
            colors.resize(12, color);

            math::Matrix3f R;
            pose->q.to_rotation_matrix(R.begin());

            for (int i = 0; i < 6; ++i) {
                verts[2 * i + 0] = pose->x + R * propulsion->rs[i];
                verts[2 * i + 1] = pose->x + R * propulsion->rs[i] + R.col(2) * (propulsion->thrusts[i] - 9.81f * 5.0f / 6.0f);
            }

            for (uint i = 0; i < 12; ++i) {
                faces[i] = i;
            }

            ogl::MeshRenderer::Ptr mr(ogl::MeshRenderer::create());
            mr->set_primitive(GL_LINES);
            mr->set_shader(shaders[VCOLOR]);
            mr->set_mesh(mesh);
            mr->draw();
        }
};

class Model {
public:
    typedef std::shared_ptr<Model> Ptr;
public:
    static Ptr create() {
        return Ptr(new Model());
    }
    math::Matrix4f m;

    struct Component {
        mve::TriangleMesh::Ptr mesh;
        ogl::MeshRenderer::Ptr mr;
        ogl::Texture::Ptr texture;
    };
    std::vector<Component> components;

    ogl::ShaderProgram::Ptr shader;
};

class Factory {
    public:
        Entity::Ptr static create_copter()
        {
            Pose::Ptr pose(new Pose);
            pose->x = math::Vec3f(0.0f, 0.0f, 20.0f);
            pose->q = math::Quatf(math::Vec3f(0.0f, 0.0f, 1.0f), 0.0f);
            std::string basename = util::fs::dirname(util::fs::get_binary_path());
            mve::TriangleMesh::Ptr mesh;
            try {
                mesh = mve::geom::load_ply_mesh(basename + "/models/proxy.ply");
            } catch (std::exception& e) {
                std::cerr << "\tCould not load mesh: "<< e.what() << std::endl;
                std::exit(EXIT_FAILURE);
            }
            MeshRenderer::Ptr mr(new MeshRenderer(pose));
            mr->set_shader(shaders[VCOLOR]); //TODO terrible design because the order matters
            mr->add_mesh(mesh, nullptr);
            Entity::Ptr ret(new Entity);
            ret->add_component(pose);
            ret->add_component(mr);
            return ret;
        }
};

class Copter {
public:
    Model::Ptr model;
    ogl::Camera fpv_cam;
    ogl::Camera cam_cam;
    struct Motor {
        math::Vec3f pos;
        float speed;
    };
    std::array<Motor, 6> motors;
    // State
    math::Vec3f x;
    math::Quaternion<float> q;
    math::Vec3f omega;
    math::Vec3f v;

    // Physics
    math::Matrix3f Ii;
    float m;
    float gamma;
    float drag_coef;

    Copter() : x(0.0f, 0.0f, 4.0f), v(0.0f, 0.0f, 0.0f),
        q(math::Vec3f(0.0f, 0.0f, 1.0f), pi / 2.0f), omega(0.0f),
        m(5.0f), gamma(0.1f), drag_coef(0.75f) {
        math::Matrix3f I;
        math::matrix_set_identity(I.begin(), 3);
        I(0, 0) = 0.25 * m;
        I(1, 1) = 0.25 * m;
        I(2, 2) = 0.5 * m;
        for (std::size_t i = 0; i < 6; ++i) {
            motors[i].speed = (9.81f * m) / 6.0f;
            float theta = (2.0f * pi) / 6.0f * i + pi / 6.0f;
            motors[i].pos = math::Vec3f(0.9 * std::cos(theta), 0.9 * std::sin(theta), 0.0f);
        }

        Ii = math::matrix_inverse(I);
    }

    void simulate(float delta) {
        float h = delta;
        math::Matrix3f R;
        q.to_rotation_matrix(R.begin());
        math::Vec3f up = R.col(2);

        float air_speed = v.norm();
        float csarea = 0.2f + 0.1f;//calculate properly

        math::Vec3f f = math::Vec3f(0.0, 0.0f, -9.81f * m);
        if (air_speed > 0.0f) {
            f += -v.normalized() * drag_coef * csarea * air_speed * air_speed;
        }
        math::Vec3f tau = math::Vec3f(0.0f);
        for (std::size_t i = 0; i < 6; ++i) {
            math::Vec3f thrust = up * motors[i].speed;
            f += thrust;
            tau += (R * -motors[i].pos).cross(thrust);
            tau += i % 2 ? -gamma * thrust : gamma * thrust;
        }

        //TODO replace Newton
        x = x + v * h;
        v = v + (f / m) * h;
        omega = omega + R * Ii * R.transposed() * tau * h;

        math::Vec3f qs = R.transposed() * omega * h;
        float theta = qs.norm();
        if (std::abs(theta) >= std::numeric_limits<float>::epsilon()) {
            math::Vec3f e(qs[0] / theta, qs[1] / theta, qs[2] / theta);
            math::Quaternion<float> qd(e, theta);
            q = qd * q;
        }
    }

    void animate() {
        math::Matrix3f rot;
        q.to_rotation_matrix(rot.begin());

        math::Vec3f view_dir = rot.col(0);
        math::Vec3f right = rot.col(1);
        math::Vec3f up = rot.col(2);
        math::Vec3f trans = x;
        model->m = rot.hstack(trans).vstack(math::Vec4f(0.0f, 0.0f, 0.0f, 1.0f));

        //TODO manual
        fpv_cam.pos = x + view_dir * 0.5f;
        fpv_cam.viewing_dir = view_dir;
        fpv_cam.up_vec = up;
        fpv_cam.update_matrices();
        cam_cam.pos = x + 1.5f * up - view_dir * 3.0f;
        cam_cam.viewing_dir = math::matrix_rotation_from_axis_angle(right, pi / 4) * view_dir;
        cam_cam.up_vec = up;
        cam_cam.update_matrices();
    }
};

std::vector<ogl::VertexArray::Ptr> primitives;
std::vector<Model::Ptr> models;
ogl::Camera scene_cam;
Copter copter;

void load_texture(std::string const & path, ogl::Texture::Ptr * texture_ptr) {
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

    *texture_ptr = texture;
}

void load_model(std::string const & path, Model::Ptr * model_ptr = nullptr) {
    std::string ext = util::string::right(path, 4);

    Model::Ptr model = Model::create();
    math::matrix_set_identity(model->m.begin(), 4);

    if (ext == ".ply") {
        mve::TriangleMesh::Ptr mesh;
        try {
            mesh = mve::geom::load_ply_mesh(path);
        } catch (std::exception& e) {
            std::cerr << "\tCould not load mesh: "<< e.what() << std::endl;
            std::exit(EXIT_FAILURE);
        }
        ShaderType shader_type = mesh->has_vertex_colors() ? VCOLOR : SURFACE;
        model->shader = shaders[shader_type];

        mesh->ensure_normals(true, true);

        model->components.emplace_back();
        Model::Component & component = model->components.back();
        component.mr = ogl::MeshRenderer::create();
        component.mesh = mesh;
        component.mr->set_mesh(mesh);
        component.mr->set_shader(model->shader);
    } else if (ext == ".obj") {
        std::vector<mve::geom::ObjModelPart> obj_model_parts;
        try {
             mve::geom::load_obj_mesh(path, &obj_model_parts);
        } catch (std::exception& e) {
            std::cerr << "Could not load model: " << e.what() << std::endl;
            exit(EXIT_FAILURE);
        }

        model->shader = shaders[TEXTURE];

        std::size_t num_components = obj_model_parts.size();

        model->components.resize(num_components);
        for (std::size_t i = 0; i < num_components; ++i) {
            Model::Component & component = model->components[i];
            component.mesh = obj_model_parts[i].mesh;
            load_texture(obj_model_parts[i].texture_filename, &component.texture);
            component.mr = ogl::MeshRenderer::create();
            component.mr->set_mesh(component.mesh);
            component.mr->set_shader(model->shader);
        }

    } else {
        std::cerr << "Unknown file extension " << ext << std::endl;
        exit(EXIT_FAILURE);
    }

    *model_ptr = model;
}

std::vector<Entity::Ptr> entities;

void init(void) {
    std::string basename = util::fs::dirname(util::fs::get_binary_path());

    shaders[VCOLOR] = ogl::ShaderProgram::create();
    if (!shaders[VCOLOR]->try_load_all(basename + "/shaders/vcolor")) {
        std::cerr << "\tCould not load shaders from: " << basename << std::endl;
        std::exit(EXIT_FAILURE);
    }
    shaders[SURFACE] = ogl::ShaderProgram::create();
    if (!shaders[SURFACE]->try_load_all(basename + "/shaders/surface")) {
        std::cerr << "\tCould not load shaders from: " << basename << std::endl;
        std::exit(EXIT_FAILURE);
    }
    shaders[TEXTURE] = ogl::ShaderProgram::create();
    if (!shaders[TEXTURE]->try_load_all(basename + "/shaders/texture")) {
        std::cerr << "\tCould not load shaders from: " << basename << std::endl;
        std::exit(EXIT_FAILURE);
    }

    primitives.push_back(ogl::create_axis_renderer(shaders[VCOLOR]));

    load_model(basename + "/models/proxy.ply", &copter.model);
    models.push_back(copter.model);

    entities.push_back(Factory::create_copter());

    scene_cam.pos = math::Vec3f(0.0f, 2.0f, 50.0f);
    scene_cam.update_matrices();

    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
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
    for (ogl::VertexArray::Ptr const & primitive : primitives) {
        primitive->draw();
    }

    for (Entity::Ptr const & entity : entities) {
        entity->render();
    }

    for (Model::Ptr const & model : models) {
        ogl::ShaderProgram::Ptr & sp = model->shader;
        sp->bind();
        sp->send_uniform("modelmat", model->m);
        sp->unbind();
        for (Model::Component const & component : model->components) {
            if (component.texture != nullptr) {
                component.texture->bind();
            }
            component.mr->draw();
        }
    }
}

void display(void) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glViewport(0, 0, 960, 1080);
    render(scene_cam);
    glViewport(960, 540, 960, 540);
    render(copter.fpv_cam);
    glViewport(960, 0, 960, 540);
    render(copter.cam_cam);

    glFlush();
}

void simulate(float delta) {
    copter.simulate(delta);
    copter.animate();
}

/* ---------------------------------------------------------------- */

int main(int argc, char **argv) {
    util::system::register_segfault_handler();
    util::system::print_build_timestamp(argv[0]);

    Arguments args = parse_args(argc, argv);

    Window window("UAVMVS Simulator", 1920, 1080);

    init();
    Model::Ptr model;
    load_model(args.model, &model);
    models.push_back(model);

    if (glfwJoystickPresent(GLFW_JOYSTICK_1)) {
        std::cout << glfwGetJoystickName(GLFW_JOYSTICK_1) << std::endl;
    }

    math::Vec3f last_error(0.0f);
    math::Vec3f last_integral(0.0f);

    double past = 0;
    while (window.good())
    {
        double now = glfwGetTime();
        float delta = now - past;
        past = now;

        math::Matrix3f R;
        copter.q.to_rotation_matrix(R.begin());
        math::Vec3f rates = R.transposed() * copter.omega;
        std::cout << std::fixed << std::setprecision(4)
            << " Altitude: " << copter.x[2]
            << " Airspeed: " << copter.v.norm()
            << " Pitch: " << std::acos(R.col(0).dot(math::Vec3f(0.0f, 0.0f, 1.0f))) - pi / 2.0f << "(" << rates[1] << ")"
            << " Roll: " << std::acos(R.col(1).dot(math::Vec3f(0.0f, 0.0f, 1.0f))) - pi / 2.0f << "(" << rates[0] << ")"
            << " Yaw: " << std::acos(R.col(0).dot(math::Vec3f(1.0f, 0.0f, 0.0f))) - pi / 2.0f << "(" << rates[2] << ")"
            << std::endl;

        simulate(delta);
        display();
        window.update();

        if (glfwJoystickPresent(GLFW_JOYSTICK_1)) {
            int count;
            const float * axes = glfwGetJoystickAxes(GLFW_JOYSTICK_1, &count);
            float turn_rate = 0.5f;
            float yaw_rate = axes[3] * turn_rate;
            float pitch_rate = -axes[1] * turn_rate;
            float roll_rate = -axes[0] * turn_rate;
            float throttle = (9.81f * 5.0f / 6.0f) * (1.0f + axes[2]);

            math::Vec3f omega(roll_rate, pitch_rate, yaw_rate);

            math::Vec3f error = R.transposed() * copter.omega - omega;

            math::Vec3f integral = last_integral + error * delta;
            last_integral = integral;

            math::Vec3f derivative = (error - last_error) / delta;

            float p = 0.5f, i = 0.03f, d = 0.04f;

            math::Vec3f out = p * error + i * integral + d * derivative;

            out[0] = std::max(-0.2f, std::min(0.2f, out[0]));
            out[1] = std::max(-0.2f, std::min(0.2f, out[1]));
            out[2] = std::max(-0.2f, std::min(0.2f, out[2]));

            //std::cout << std::fixed << std::setprecision(4) << std::abs(out[0]) << " " << std::abs(out[1]) << " " << std::abs(out[2]) << std::endl;
            copter.motors[0].speed = throttle * (1.0f - out[2]) * (1.0f + 0.25f * out[0]) * (1.0f - out[1]);
            copter.motors[1].speed = throttle * (1.0f + out[2]) * (1.0f + 0.50f * out[0]);
            copter.motors[2].speed = throttle * (1.0f - out[2]) * (1.0f + 0.25f * out[0]) * (1.0f + out[1]);
            copter.motors[3].speed = throttle * (1.0f + out[2]) * (1.0f - 0.25f * out[0]) * (1.0f + out[1]);
            copter.motors[4].speed = throttle * (1.0f - out[2]) * (1.0f - 0.50f * out[0]);
            copter.motors[5].speed = throttle * (1.0f + out[2]) * (1.0f - 0.25f * out[0]) * (1.0f - out[1]);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return EXIT_SUCCESS;
}
