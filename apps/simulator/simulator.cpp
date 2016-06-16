#include <iostream>
#include <array>
#include <mutex>
#include <condition_variable>
#include <random>
#include <tuple>

#include "util/arguments.h"
#include "util/file_system.h"
#include "util/system.h"
#include "util/exception.h"

#include "mve/mesh_io_ply.h"
#include "mve/mesh_io_obj.h"
#include "mve/mesh_tools.h"
#include "mve/image_io.h"
#include "mve/image_tools.h"

#include "ogl/mesh_renderer.h"
#include "ogl/render_tools.h"
#include "ogl/texture.h"
#include "ogl/camera.h"

#include "acc/kd_tree.h"

#include "window.h"
#include "entity.h"

struct Arguments {
    std::string model;
    std::string proxy;
    std::string aabb;
};

Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_maxnum(3);
    args.set_nonopt_minnum(3);
    args.set_helptext_indent(28);
    args.set_usage("Usage: " + std::string(argv[0]) + " [OPTS] MODEL PROXY AABB");
    args.set_description("TODO");
    args.parse(argc, argv);

    Arguments conf;
    conf.model = args.get_nth_nonopt(0);
    conf.proxy = args.get_nth_nonopt(1);
    conf.aabb = args.get_nth_nonopt(2);

    for (util::ArgResult const* i = args.next_option(); i != 0; i = args.next_option()) {
        switch (i->opt->sopt) {
        default:
            throw std::invalid_argument("Invalid option");
        }
    }
    return conf;
}

constexpr float pi = std::acos(-1.0f);

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
            if (pose != nullptr) {
                math::Matrix3f rot;
                pose->q.to_rotation_matrix(rot.begin());
                math::Vec3f trans = pose->x;
                math::Matrix4f m = rot.hstack(trans).vstack(math::Vec4f(0.0f, 0.0f, 0.0f, 1.0f));
                shader->bind();
                shader->send_uniform("modelmat", m);
                shader->unbind();
            }

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

            for (uint i = 0; i < num_engines; ++i) {
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


class Factory {
    public:
        Entity::Ptr static create_copter(std::array<ogl::ShaderProgram::Ptr, 4> const & shaders,
            Pose::Ptr pose, Trajectory::Ptr trajectory)
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
            pose->x = math::Vec3f(0.0f, 0.0f, 30.0f);
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
            trajectory = Trajectory::Ptr(new Trajectory);
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

        Entity::Ptr static create_model(std::array<ogl::ShaderProgram::Ptr, 4> const & shaders,
            std::string const & path)
        {
            Entity::Ptr ret(new Entity);
            Model::Ptr model = load_model(path);
            ModelRenderer::Ptr mr(new ModelRenderer(nullptr, shaders[model->shader_type]));
            for (Model::Part const & part : model->parts) {
                mr->add_mesh(part.mesh, part.texture);
            }
            ret->add_component(mr);
            return ret;
        }

#if 0
        Entity::Ptr static create_trajectory(mve::TriangleMesh::Ptr aabb, acc::KDTree<3> const & tree) {
            Pose::Ptr pose = Pose::Ptr(new Pose);
            pose->x = math::Vec3f(0.0f, 0.0f, 30.0f);
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
            FlightController::Ptr fc(new FlightController(pose, motion, control, propulsion));


            math::Vec3f min, max;
            mve::geom::mesh_find_aabb(aabb, min, max);
            std::random_device rd;
            std::mt19937 gen(rd());
            std::discrete_distribution<> d({10, 20, 40, 20, 10});

            control->throttle = 1.0f;
            float delta_time = 0.5f;
            for (float time = 0.0f; time < 100.0f; time += delta_time) {

                control->omega[0] = ((d(gen) - 2) / 2.0f);
                control->omega[1] = ((d(gen) - 2) / 2.0f);
                control->omega[2] = ((d(gen) - 2) / 2.0f);
                ps->update(delta_time);
                l->update(delta_time);
                fc->update(delta_time);
                for (int d = 0; d < 3; ++d) {
                    if (pose->x[d] < min[d] || max[d] < pose->x[d]) break;
                }
                if (tree.find_nn(pose->x, 5.0f).second < 5.0f) break;
            }

            std::cout << trajectory->xs.size() << std::endl;

            TrajectoryRenderer::Ptr tr(new TrajectoryRenderer(trajectory, shaders[LINES]));
            Entity::Ptr ret(new Entity);
            ret->add_component(pose);
            ret->add_component(trajectory);
            ret->add_component(tr);
            return ret;
        }
#endif

};

void
load_shader(std::string const & path, ogl::ShaderProgram::Ptr shader) {
    shader = ogl::ShaderProgram::create();
    if (!shader->try_load_all(path)) {
        std::cerr << "\tCould not load shaders from: " << path << std::endl;
        std::exit(EXIT_FAILURE);
    }
}

void
load_shaders(std::array<ogl::ShaderProgram::Ptr, 4> * shaders) {
    std::string basename = util::fs::dirname(util::fs::get_binary_path());
    load_shader(basename + "/shaders/vcolor", shaders->at(VCOLOR));
    load_shader(basename + "/shaders/surface", shaders->at(SURFACE));
    load_shader(basename + "/shaders/texture", shaders->at(TEXTURE));
    load_shader(basename + "/shaders/lines", shaders->at(LINES));
}

void
init_opengl(void) {
    glClearColor(0.9f, 0.9f, 0.9f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void render(std::vector<Entity::Ptr> const & entities,
    std::array<ogl::ShaderProgram::Ptr, 4> shaders,
    ogl::Camera const & cam)
{
    for (ogl::ShaderProgram::Ptr & sp : shaders) {
        sp->bind();
        sp->send_uniform("viewmat", cam.view);
        sp->send_uniform("projmat", cam.proj);
        math::Matrix4f eye;
        math::matrix_set_identity(eye.begin(), 4);
        sp->send_uniform("modelmat", eye);
        sp->unbind();
    }
    for (Entity::Ptr const & entity : entities) {
        entity->render();
    }
    glEnable(GL_BLEND);
    //aabb->render();
    glDisable(GL_BLEND);
}

void display(std::vector<Entity::Ptr> const & entities,
    std::array<ogl::ShaderProgram::Ptr, 4> const & shaders,
    Pose::Ptr pose)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    ogl::Camera camera;

    glViewport(0, 0, 960, 1080);
    camera.right = 0.05f;
    camera.top = 0.05f / (960.0f / 1080.0f);
    camera.pos = math::Vec3f(0.0f, 0.0f, 150.0f);
    camera.update_matrices();
    render(entities, shaders, camera);

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
    render(entities, shaders, camera);

    glViewport(960, 0, 960, 540);
    camera.pos = pose->x + 1.5f * up - view_dir * 2.0f;
    camera.viewing_dir = math::matrix_rotation_from_axis_angle(right, pi / 4) * view_dir;
    camera.up_vec = up;
    camera.update_matrices();
    render(entities, shaders, camera);

    glFlush();
}

void simulate(std::vector<Entity::Ptr> const & entities, float delta) {
    for (Entity::Ptr const & entity : entities) {
        entity->update(delta);
    }
}

int main(int argc, char **argv) {
    util::system::register_segfault_handler();
    util::system::print_build_timestamp(argv[0]);

    Arguments args = parse_args(argc, argv);

    Window window("UAVMVS Simulator", 1920, 1080);
    init_opengl();

    std::array<ogl::ShaderProgram::Ptr, 4> shaders;
    load_shaders(&shaders);

    std::vector<Entity::Ptr> entities;
    Trajectory::Ptr trajecory;
    Pose::Ptr pose;
    entities.push_back(Factory::create_copter(shaders, pose, trajectory));
    entities.push_back(Factory::create_model(shaders, args.model));

    mve::TriangleMesh::Ptr aabb;
    try {
        aabb = mve::geom::load_ply_mesh(args.aabb);
    } catch (std::exception& e) {
        std::cerr << "\tCould not load mesh: "<< e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }

    mve::TriangleMesh::Ptr mesh;
    try {
        mesh = mve::geom::load_ply_mesh(args.proxy);
    } catch (std::exception& e) {
        std::cerr << "\tCould not load mesh: "<< e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }

    std::vector<math::Vec3f> const & verts = mesh->get_vertices();
    std::cout << "Building acceleration structure... " << std::flush;
    acc::KDTree<3, uint> tree(verts);
    std::cout << "done." << std::endl;

    if (glfwJoystickPresent(GLFW_JOYSTICK_1)) {
        std::cout << glfwGetJoystickName(GLFW_JOYSTICK_1) << std::endl;
    }

    double past = glfwGetTime();
    while (window.good())
    {
        double now = glfwGetTime();
        float delta = now - past;
        past = now;

        simulate(entities, delta);
        display(entities, shaders, pose);

#if 1
        uint nn;
        float dist;
        std::tie(nn, dist) = tree.find_nn(pose->x, 5.0f);
        if (nn != -1) std::cout << "Proximity alert " << dist << "m" << std::endl;
#endif

        window.update();

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (!args.trajectory.empty()) {
        trajectory->save(args.trajectory)
    }

    return EXIT_SUCCESS;
}
