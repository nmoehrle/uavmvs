#include <iostream>
#include <fstream>
#include <array>
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
#include "mve/camera.h"

#include "ogl/mesh_renderer.h"
#include "ogl/render_tools.h"
#include "ogl/texture.h"
#include "ogl/camera.h"

#include "acc/kd_tree.h"

#include "sim/window.h"
#include "sim/entity.h"
#include "sim/entities/logger.h"
#include "sim/entities/receiver.h"
#include "sim/entities/flight_controller.h"
#include "sim/entities/physics_simulator.h"
#include "sim/entities/trajectory_renderer.h"
#include "sim/entities/propulsion_renderer.h"
#include "sim/engine.h"

#include "utp/trajectory_io.h"

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

class Simulator : public Engine {
private:
    std::string basepath;

public:
    typedef std::shared_ptr<Simulator> Ptr;

    Simulator() : Engine() {}

    Entity::Ptr create_copter(Pose::ConstPtr * pose_ptr, Trajectory::ConstPtr * trajectory_ptr)
    {
        mve::TriangleMesh::Ptr mesh;
        try {
            std::string path = util::fs::join_path(__ROOT__, "res/meshes/copter.ply");
            mesh = mve::geom::load_ply_mesh(path);
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
    glEnable(GL_FRAMEBUFFER_SRGB);
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

#if 0
    std::vector<math::Vec3f> const & verts = mesh->get_vertices();
    std::cout << "Building acceleration structure... " << std::flush;
    acc::KDTree<3, uint> tree(verts);
    std::cout << "done." << std::endl;
#endif

    if (glfwJoystickPresent(GLFW_JOYSTICK_1)) {
        std::cout << glfwGetJoystickName(GLFW_JOYSTICK_1) << std::endl;
    }

    double past = glfwGetTime();
    double last = 0.0f;
    while (window.good())
    {
        double now = glfwGetTime();
        double delta_time = now - past;
        past = now;

        simulator->update(delta_time);
        if (last >= 0.01f) {
            last = 0.0f;
            display(simulator, pose);
            window.update();
        } else {
            last += delta_time;
        }

#if 0
        std::pair<uint, float> nn;
        if (tree.find_nn(pose->x, &nn, 5.0f)) {
            std::cout << "Proximity alert " << nn.second << "m" << std::endl;
        }
#endif

        int rest = 4 - (int)(1000 * delta_time);
        std::this_thread::sleep_for(std::chrono::milliseconds(rest));
    }

    if (!args.trajectory.empty()) {
        std::vector<mve::CameraInfo> tmp;
        std::size_t length = trajectory->xs.size() & trajectory->qs.size();
        tmp.resize(length);
        for (std::size_t i = 0; i < length; ++i) {
            mve::CameraInfo & cam = tmp.at(i);
            math::Vec3f pos = trajectory->xs[i];
            math::Matrix3f rot;
            trajectory->qs[i].to_rotation_matrix(rot.begin());
            math::Vec3f trans = -rot * pos;
            std::copy(trans.begin(), trans.end(), cam.trans);
            std::copy(rot.begin(), rot.end(), cam.rot);
            cam.flen = 0.86f;
        }
        utp::save_trajectory(tmp, args.trajectory);
    }

    return EXIT_SUCCESS;
}
