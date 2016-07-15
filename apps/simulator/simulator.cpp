#include <iostream>
#include <fstream>
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

#include "sim/window.h"
#include "sim/entity.h"
#include "sim/engine.h"

#define __ABSFILE__ util::fs::join_path(__ROOT__, __FILE__)

struct Arguments {
    std::string model;
    std::string proxy;
    std::string trajectory;
};

Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_maxnum(2);
    args.set_nonopt_minnum(2);
    args.set_helptext_indent(28);
    args.set_usage("Usage: " + std::string(argv[0]) + " [OPTS] MODEL PROXY");
    args.add_option('e', "export-trajectory", true, "export trajectory to file");
    args.set_description("Hexacopter simulator");
    args.parse(argc, argv);

    Arguments conf;
    conf.model = args.get_nth_nonopt(0);
    conf.proxy = args.get_nth_nonopt(1);

    for (util::ArgResult const* i = args.next_option(); i != 0; i = args.next_option()) {
        switch (i->opt->sopt) {
        case 'e':
            conf.trajectory = i->arg;
        break;
        default:
            throw std::invalid_argument("Invalid option");
        }
    }
    return conf;
}

constexpr float pi = std::acos(-1.0f);

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


class Simulator : public Engine {
private:
    std::string basepath;

public:
    typedef std::shared_ptr<Simulator> Ptr;

    Simulator(std::string const & basepath = util::fs::dirname(__ABSFILE__))
        : basepath(basepath), Engine() {}

    Entity::Ptr create_copter(Pose::ConstPtr * pose_ptr, Trajectory::ConstPtr * trajectory_ptr)
    {
        mve::TriangleMesh::Ptr mesh;
        try {
            mesh = mve::geom::load_ply_mesh(basepath + "/models/proxy.ply");
        } catch (std::exception & e) {
            throw std::runtime_error(std::string("Could not load mesh: ") + e.what());
        } //TODO extract

        Pose::Ptr pose(new Pose);
        pose->x = math::Vec3f(0.0f, 0.0f, 30.0f);
        pose->q = math::Quatf(math::Vec3f(0.0f, 1.0f, 0.0f), 0.0f);

        Motion::Ptr motion(new Motion);
        motion->v = math::Vec3f(0.0f);
        motion->omega = math::Vec3f(0.0f);

        Physics::Ptr physics(new Physics);
        physics->mass = 7.51f;
        math::Matrix3f I(0.0f);
        I(0, 0) = 0.25 * physics->mass;
        I(1, 1) = 0.25 * physics->mass;
        I(2, 2) = 0.5 * physics->mass;
        physics->Ii = math::matrix_inverse(I);
        physics->drag_coeff = 0.75f;

        Propulsion::Ptr propulsion(new Propulsion);
        propulsion->max_thrust = 9.81f * 2.85f;
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

        DynamicModelRenderer::Ptr mr(new DynamicModelRenderer(pose, this->get_shader(VCOLOR)));
        mr->add_mesh(mesh, nullptr);
        TrajectoryRenderer::Ptr tr(new TrajectoryRenderer(trajectory, this->get_shader(LINES)));
        PropulsionRenderer::Ptr pr(new PropulsionRenderer(pose, physics, propulsion, this->get_shader(LINES)));

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

        *pose_ptr = pose;
        *trajectory_ptr = trajectory;

        this->add_entity(ret);

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
init_opengl(void) {
    glClearColor(0.9f, 0.9f, 0.9f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void display(Simulator::Ptr simulator, Pose::ConstPtr pose)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    ogl::Camera camera;

    math::Matrix3f rot;
    pose->q.to_rotation_matrix(rot.begin());
    math::Vec3f view_dir = rot.col(0);
    math::Vec3f right = rot.col(1);
    math::Vec3f up = rot.col(2);

    glViewport(0, 0, 1920, 1080);
    camera.top = 0.05f;
    camera.right = 0.05f * (960.0f / 540.0f);
    camera.pos = pose->x + view_dir * 0.85f;
    camera.viewing_dir = math::matrix_rotation_from_axis_angle(right, - pi / 8.0f) * view_dir;
    camera.up_vec = up;
    camera.update_matrices();
    simulator->render(camera);

    glViewport(1280, 600, 640, 480);
    glScissor(1280, 600, 640, 480);
    glEnable(GL_SCISSOR_TEST);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDisable(GL_SCISSOR_TEST);
    camera.pos = pose->x + 1.5f * up - view_dir * 2.0f;
    camera.viewing_dir = math::matrix_rotation_from_axis_angle(right, pi / 4.0f) * view_dir;
    camera.up_vec = up;
    camera.update_matrices();
    simulator->render(camera);

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

    Pose::ConstPtr pose;
    Trajectory::ConstPtr trajectory;
    Simulator::Ptr simulator(new Simulator());
    simulator->create_copter(&pose, &trajectory);
    simulator->create_static_model(args.model);

#if 0
    mve::TriangleMesh::Ptr aabb;
    try {
        aabb = mve::geom::load_ply_mesh(args.aabb);
    } catch (std::exception& e) {
        std::cerr << "\tCould not load mesh: "<< e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }
#endif

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
        double delta_time = now - past;
        past = now;

        simulator->update(delta_time);
        display(simulator, pose);

        uint nn;
        float dist;
        std::tie(nn, dist) = tree.find_nn(pose->x, 5.0f);
        if (nn != -1) std::cout << "Proximity alert " << dist << "m" << std::endl;

        window.update();

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (!args.trajectory.empty()) {
        std::ofstream out(args.trajectory.c_str());
        if (!out.good()) throw std::runtime_error("Could not open file");
        for (std::size_t i = 0; i < trajectory->xs.size(); ++i) {
            out << trajectory->xs[i] << std::endl;

            math::Matrix3f rot;
            trajectory->qs[i].to_rotation_matrix(rot.begin());
            out << rot;
        }
        out.close();
    }

    return EXIT_SUCCESS;
}
