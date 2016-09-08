#include <memory>
#include <iostream>

#include "util/system.h"
#include "util/arguments.h"
#include "util/file_system.h"

#include "math/matrix_tools.h"

#include "mve/volume.h"
#include "mve/image_io.h"
#include "mve/marching_cubes.h"
#include "mve/mesh_io_ply.h"

#include "ogl/events.h"
#include "ogl/camera.h"
#include "ogl/camera_trackball.h"
#include "ogl/mesh_renderer.h"

#include "sim/window.h"
#include "sim/shader.h"
#include "sim/shader_type.h"

struct Arguments {
    std::string path;
};

Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_minnum(1);
    args.set_nonopt_maxnum(1);
    args.set_usage("Usage: " + std::string(argv[0]) + " [OPTS] PATH");
    args.set_description("Visualizer");
    args.parse(argc, argv);

    Arguments conf;
    conf.path = args.get_nth_nonopt(0);

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

mve::TriangleMesh::Ptr extract(mve::FloatVolume::Ptr volume, float iso) {
    std::vector<float> & voxels = volume->get_data();
    std::for_each(voxels.begin(), voxels.end(), [iso] (float & value) { value -= iso; });
    mve::VolumeMCAccessor accessor;
    accessor.vol = volume;
    mve::TriangleMesh::Ptr mesh = mve::geom::marching_cubes(accessor);
    mesh->ensure_normals(true, true);
    return mesh;
}

int main(int argc, char **argv) {
    util::system::register_segfault_handler();
    util::system::print_build_timestamp(argv[0]);

    Arguments args = parse_args(argc, argv);

    mve::FloatVolume::Ptr volume;
    {
        mve::FloatImage::Ptr image;
        image = std::dynamic_pointer_cast<mve::FloatImage>(mve::image::load_mvei_file(args.path));
        volume = mve::FloatVolume::create(image->width(), image->height(), image->channels());
        std::vector<float> & data = volume->get_data();
        std::swap(data, image->get_data());
    }

    Window window("Visualizer", 1920, 1080);
    init_opengl();

    Shader::Ptr shader(new Shader());
    std::string path = util::fs::join_path(__ROOT__, "res/shaders");
    shader->load_shader_program(path + "/" + shader_names[SURFACE]);
    math::Matrix4f eye;
    math::matrix_set_identity(eye.begin(), 4);
    shader->set_model_matrix(eye);
    shader->update();

    float iso = 0.5f;

    ogl::MeshRenderer::Ptr renderer = ogl::MeshRenderer::create();
    renderer->set_shader(shader->get_shader_program());
    renderer->set_mesh(extract(volume, iso));

    ogl::Camera camera;
    ogl::CamTrackball trackball;
    camera.width = 1920;
    camera.height = 1080;
    camera.right = (1920.0f / 1080.0f) * camera.top;
    trackball.set_camera(&camera);
    trackball.set_camera_params(math::Vec3f(-2.0f, 2.0f, 2.0f),
        math::Vec3f(-2.0f, 2.0f, 0.0f), math::Vec3f(0.0f, 1.0f, 0.0f));

    ogl::MouseEvent event;
    window.register_key_callback(333, [volume, renderer, &iso] (int action) {
        if (!action) return;
        iso -= 0.01f;
        std::cout << iso << std::endl;
        renderer->set_mesh(extract(volume, -0.01f));
    });
    window.register_key_callback(334, [volume, renderer, &iso] (int action) {
        if (!action) return;
        iso += 0.01f;
        std::cout << iso << std::endl;
        renderer->set_mesh(extract(volume, +0.01f));
    });
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
        renderer->draw();

        glFlush();

        window.update();

        int rest = 4 - (int)(1000 * delta_time);
        std::this_thread::sleep_for(std::chrono::milliseconds(rest));
    }

    return EXIT_SUCCESS;
}
