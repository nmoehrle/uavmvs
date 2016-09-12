#include <memory>
#include <iostream>

#include "util/system.h"
#include "util/arguments.h"
#include "util/file_system.h"

#include "util/trajectory_io.h"
#include "util/matrix_io.h"

#include "math/matrix_tools.h"

#include "mve/image_io.h"
#include "mve/marching_cubes.h"
#include "mve/mesh_io_ply.h"
#include "math/bspline.h"

#include "ogl/events.h"
#include "ogl/camera.h"
#include "ogl/camera_trackball.h"
#include "ogl/mesh_renderer.h"

#include "sim/engine.h"
#include "sim/window.h"
#include "sim/shader.h"
#include "sim/shader_type.h"
#include "sim/entities/trajectory_renderer.h"

struct Arguments {
    std::string path;
    std::string trajectory;
};

Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_minnum(2);
    args.set_nonopt_maxnum(2);
    args.set_usage("Usage: " + std::string(argv[0]) + " [OPTS] MESH TRAJECTORY");
    args.set_description("Visualizer");
    args.parse(argc, argv);

    Arguments conf;
    conf.path = args.get_nth_nonopt(0);
    conf.trajectory = args.get_nth_nonopt(1);

    for (util::ArgResult const* i = args.next_option();
         i != 0; i = args.next_option()) {
        switch (i->opt->sopt) {
        default:
            throw std::invalid_argument("Invalid option");
        }
    }

    return conf;
}

void
init_opengl(void) {
    glClearColor(0.9f, 0.9f, 0.9f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

int main(int argc, char **argv) {
    util::system::register_segfault_handler();
    util::system::print_build_timestamp(argv[0]);

    Arguments args = parse_args(argc, argv);

    Window window("Visualizer", 1920, 1080);
    init_opengl();

    Engine::Ptr engine(new Engine());
    engine->create_static_model(args.path);

    Shader::Ptr shader(new Shader());
    std::string path = util::fs::join_path(__ROOT__, "res/shaders");
    shader->load_shader_program(path + "/" + shader_names[LINES]);
    math::Matrix4f eye;
    math::matrix_set_identity(&eye);
    shader->set_model_matrix(eye);

    math::BSpline<math::Vec3f> spline;
    spline.set_degree(3);
    {
        std::vector<mve::CameraInfo> trajectory;
        load_trajectory(args.trajectory, &trajectory);

        for (std::size_t i = 0; i < trajectory.size(); ++i) {
            mve::CameraInfo const & cam = trajectory[i];
            math::Vec3f trans(cam.trans);
            math::Matrix3f rot(cam.rot);
            math::Vec3f pos = -rot.transposed() * trans;
            spline.add_point(pos);
        }
    }
    spline.uniform_knots(0.0, 1.0f);

    Trajectory::Ptr trajectory(new Trajectory);
    for (float t = 0.0f; t < 1.0f; t += 0.001f) {
        trajectory->xs.push_back(spline.evaluate(t));
        trajectory->qs.emplace_back(math::Vec3f(0.0f, 0.0f, 1.0f), 0.0f);
    }

    Pose::Ptr pose = Pose::Ptr(new Pose);
    TrajectoryRenderer::Ptr tr(new TrajectoryRenderer(trajectory, shader));
    Entity::Ptr ret(new Entity);
    ret->add_component(pose);
    ret->add_component(trajectory);
    ret->add_component(tr);
    engine->add_entity(ret);

    ogl::Camera camera;
    ogl::CamTrackball trackball;
    camera.width = 1920;
    camera.height = 1080;
    camera.right = (1920.0f / 1080.0f) * camera.top;
    trackball.set_camera(&camera);
    trackball.set_camera_params(math::Vec3f(-2.0f, 2.0f, 2.0f),
        math::Vec3f(-2.0f, 2.0f, 0.0f), math::Vec3f(0.0f, 1.0f, 0.0f));

    ogl::MouseEvent event;

    window.register_mouse_button_callback(0, [&trackball, &event] (int action) {
        event.button = ogl::MOUSE_BUTTON_LEFT;
        if (action) {
            event.type = ogl::MOUSE_EVENT_PRESS;
            event.button_mask |= ogl::MOUSE_BUTTON_LEFT;
        } else {
            event.type = ogl::MOUSE_EVENT_RELEASE;
            event.button_mask &= ~ogl::MOUSE_BUTTON_LEFT;
        }
        trackball.consume_event(event);
    });
    window.register_mouse_button_callback(1, [&trackball, &event] (int action) {
        event.button = ogl::MOUSE_BUTTON_RIGHT;
        if (action) {
            event.type = ogl::MOUSE_EVENT_PRESS;
            event.button_mask |= ogl::MOUSE_BUTTON_RIGHT;
        } else {
            event.type = ogl::MOUSE_EVENT_RELEASE;
            event.button_mask &= ~ogl::MOUSE_BUTTON_RIGHT;
        }
        trackball.consume_event(event);
    });
    window.register_cursor_position_callback([&trackball, &event] (double xpos, double ypos) {
        event.type = ogl::MOUSE_EVENT_MOVE;
        event.button = ogl::MOUSE_BUTTON_NONE;
        event.x = xpos;
        event.y = ypos;
        trackball.consume_event(event);
    });
    window.register_scroll_callback([&trackball] (double, double yoffset) {
        ogl::MouseEvent event;
        event.type = (yoffset > 0.0f) ? ogl::MOUSE_EVENT_WHEEL_UP : ogl::MOUSE_EVENT_WHEEL_DOWN;
        trackball.consume_event(event);
    });

    double past = glfwGetTime();
    while (window.good())
    {
        double now = glfwGetTime();
        double delta_time = now - past;
        past = now;

        camera.pos = trackball.get_campos();
        camera.viewing_dir = trackball.get_viewdir();
        camera.up_vec = trackball.get_upvec();
        camera.update_matrices();

        shader->set_view_matrix(camera.view);
        shader->set_proj_matrix(camera.proj);
        shader->update();

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glViewport(0, 0, 1920, 1080);
        engine->render(camera);

        glFlush();

        window.update();

        int rest = 4 - (int)(1000 * delta_time);
        std::this_thread::sleep_for(std::chrono::milliseconds(rest));
    }

    return EXIT_SUCCESS;
}
