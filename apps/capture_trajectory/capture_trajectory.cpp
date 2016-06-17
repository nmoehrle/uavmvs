#include <iostream>

#include "util/system.h"
#include "util/arguments.h"

#include "mve/scene.h"
#include "mve/mesh_io_ply.h"
#include "mve/mesh_io_obj.h"
#include "mve/image_tools.h"
#include "mve/image_io.h"

#include "ogl/camera.h"
#include "ogl/check_gl_error.h"

#include "sim/window.h"
#include "sim/engine.h"

struct Arguments {
    std::string scene_dir;
    std::string model;
    std::string image_name = "undistorted";
    int width = 1920;
    int height = 1080;
};

Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_maxnum(2);
    args.set_nonopt_minnum(2);
    args.set_helptext_indent(28);
    args.set_usage("Usage: " + std::string(argv[0]) + " [OPTS] SCENE_DIR MODEL");
    args.set_description("Renders the model from all trajectory views given as scene");
    args.parse(argc, argv);

    Arguments conf;
    conf.scene_dir = args.get_nth_nonopt(0);
    conf.model = args.get_nth_nonopt(1);

    for (util::ArgResult const* i = args.next_option(); i != 0; i = args.next_option()) {
        switch (i->opt->sopt) {
        default:
            throw std::invalid_argument("Invalid option");
        }
    }
    return conf;
}

void
fill_ogl_camera(mve::CameraInfo const & camera_info, int width, int height, float znear, float zfar, ogl::Camera * ogl_camera) {
    /* Get all parameters and check them. */
    float const dimension_aspect = static_cast<float>(width) / height;
    float const pixel_aspect = camera_info.paspect;
    float const image_aspect = dimension_aspect * pixel_aspect;
    float const focal_length = camera_info.flen;
    float const ppx = camera_info.ppoint[0];
    float const ppy = camera_info.ppoint[1];

    /* Fill OpenGL view matrix */
    camera_info.fill_world_to_cam(ogl_camera->view.begin());
    camera_info.fill_cam_to_world(ogl_camera->inv_view.begin());

    /* Construct OpenGL projection matrix. */
    math::Matrix4f& proj = ogl_camera->proj;
    proj.fill(0.0f);
    proj[0] = 2.0f * focal_length
        * (image_aspect > 1.0f ? 1.0f : 1.0f / image_aspect);
    proj[2] = -2.0f * (0.5f - ppx);
    proj[5] = -2.0f * focal_length
        * (image_aspect > 1.0f ? image_aspect : 1.0f);
    proj[6] = -2.0f * (ppy - 0.5f);
    proj[10] = -(-zfar - znear) / (zfar - znear);
    proj[11] = -2.0f * zfar * znear / (zfar - znear);
    proj[14] = 1.0f;

    camera_info.fill_camera_pos(ogl_camera->pos.begin());

    ogl_camera->z_near = znear;
    ogl_camera->z_far = zfar;
}

void setup_fbo(GLuint *fbo, GLuint *rbo_color, GLuint *rbo_depth, int msaa_samples, int width, int height)
{
    /* Color renderbuffer. */
    glGenRenderbuffers(1, rbo_color);
    glBindRenderbuffer(GL_RENDERBUFFER, *rbo_color);
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, msaa_samples, GL_RGBA32F, width, height);
    ogl::check_gl_error();
    glBindRenderbuffer(GL_RENDERBUFFER, 0);

    /* Depth renderbuffer. */
    glGenRenderbuffers(1, rbo_depth);
    glBindRenderbuffer(GL_RENDERBUFFER, *rbo_depth);
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, msaa_samples, GL_DEPTH_COMPONENT32F, width, height);
    ogl::check_gl_error();
    glBindRenderbuffer(GL_RENDERBUFFER, 0);

    /* Framebuffer */
    glGenFramebuffers(1, fbo);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, *fbo);
    glFramebufferRenderbuffer(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, *rbo_color);
    ogl::check_gl_error();
    glFramebufferRenderbuffer(GL_DRAW_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, *rbo_depth);
    ogl::check_gl_error();
    GLenum fb_status = glCheckFramebufferStatus(GL_DRAW_FRAMEBUFFER);
    if (fb_status != GL_FRAMEBUFFER_COMPLETE) {
        throw std::runtime_error("Could not create framebuffer");
    }
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
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

    Window window("", 640, 480);
    init_opengl();

    Engine::Ptr engine(new Engine());
    engine->create_static_model(args.model);

    int width = args.width;
    int height = args.height;

    mve::Scene::Ptr scene;
    try {
        scene = mve::Scene::create(args.scene_dir);
    } catch (std::exception& e) {
        std::cerr << "Could not open scene: " << e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }

    GLuint fbo, rbo_color, rbo_depth;
    setup_fbo(&fbo, &rbo_color, &rbo_depth, 4, width, height);

    GLuint fbo_final, rbo_color_final, rbo_depth_final;
    setup_fbo(&fbo_final, &rbo_color_final, &rbo_depth_final, 0, width, height);

    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fbo);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, fbo);

    for (mve::View::Ptr const & view : scene->get_views()) {
        ogl::Camera ogl_cam;
        float const znear = 0.1f;
        float const zfar = 1000.0f;
        fill_ogl_camera(view->get_camera(), width, height, znear, zfar, &ogl_cam);

        glViewport(0, 0, width, height);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        engine->render(ogl_cam);
        glFlush();

        /* Bind multisampled framebuffer image to default framebuffer. */
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fbo_final);
        glBindFramebuffer(GL_READ_FRAMEBUFFER, fbo);
        glBlitFramebuffer(0, 0, width, height, 0, 0, width, height, GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT, GL_NEAREST);
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fbo);
        glBindFramebuffer(GL_READ_FRAMEBUFFER, fbo_final);

        /* Read image from OpenGL */
        mve::FloatImage::Ptr image = mve::FloatImage::create(width, height, 4);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadPixels(0, 0, width, height, GL_RGBA, GL_FLOAT, image->begin());
        ogl::check_gl_error();
        mve::image::flip<float>(image, mve::image::FLIP_VERTICAL);

        image->delete_channel(3);

        view->set_image(mve::image::float_to_byte_image(image), args.image_name);
        view->save_view();
    }

    return EXIT_SUCCESS;
}
