#include <chrono>
#include <iostream>

#include "util/system.h"
#include "util/arguments.h"
#include "util/tokenizer.h"

#include "mve/scene.h"
#include "mve/depthmap.h"
#include "mve/image_io.h"
#include "mve/image_tools.h"
#include "mve/mesh_io_ply.h"

#include "ogl/camera.h"
#include "ogl/mesh_renderer.h"
#include "ogl/check_gl_error.h"

#include "sim/window.h"

#include "acc/bvh_tree.h"

#include "cacc/util.h"
#include "cacc/image.h"
#include "cacc/matrix.h"
#include "cacc/bvh_tree.h"
#include "cacc/tracing.h"

#include <cuda_gl_interop.h>

typedef unsigned int uint;
typedef acc::BVHTree<uint, math::Vec3f> BVHTree;

const char *fragment_shader = R"(
#version 330 core

layout(location=0) out float depth;

uniform float znear;
uniform float zfar;

void main(void)
{
    gl_FragDepth = gl_FragCoord.z;
    depth = (zfar * znear) / ((znear - zfar) * gl_FragCoord.z + zfar);
}
)";

const char *vertex_shader = R"(
#version 330 core

in vec4 pos;

uniform mat4 viewmat;
uniform mat4 projmat;

void main(void)
{
    gl_Position = projmat * (viewmat * pos);
}
)";

struct Arguments {
    std::string scene_dir;
    std::string mesh;
    std::string image_name = "original";
    int width = 1920;
    int height = 1080;
};

Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_maxnum(2);
    args.set_nonopt_minnum(2);
    args.set_helptext_indent(28);
    args.set_usage("Usage: " + std::string(argv[0]) + " [OPTS] SCENE MESH");
    args.set_description("Test app for cuda and opengl interoperability");
    args.add_option('r', "resolution", true, "resolution [1920x1080]");
    args.parse(argc, argv);

    Arguments conf;
    conf.scene_dir = args.get_nth_nonopt(0);
    conf.mesh = args.get_nth_nonopt(1);

    for (util::ArgResult const* i = args.next_option(); i != 0; i = args.next_option()) {
        switch (i->opt->sopt) {
        case 'r':
        {
            util::Tokenizer tok;
            tok.split(i->arg, 'x');
            if (tok.size() != 2) throw std::invalid_argument("Invalid resolution");
            conf.width = tok.get_as<int>(0);
            conf.height =  tok.get_as<int>(1);
        }
        break;
        default:
            throw std::invalid_argument("Invalid option");
        }
    }
    return conf;
}

/* TODO extract and merge with capture_trajectory. */
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

void setup_fbo(GLuint *fbo, GLuint *rbo, GLuint * dbo, int width, int height)
{
    glGenRenderbuffers(1, rbo);
    glBindRenderbuffer(GL_RENDERBUFFER, *rbo);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_R32F, width, height);
    ogl::check_gl_error();
    glBindRenderbuffer(GL_RENDERBUFFER, 0);

    glGenRenderbuffers(1, dbo);
    glBindRenderbuffer(GL_RENDERBUFFER, *dbo);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, width, height);
    ogl::check_gl_error();
    glBindRenderbuffer(GL_RENDERBUFFER, 0);

    glGenFramebuffers(1, fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, *fbo);
    glFramebufferRenderbuffer(
        GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, *rbo);
    glFramebufferRenderbuffer(
        GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, *dbo);
    ogl::check_gl_error();

    if(GL_FRAMEBUFFER_COMPLETE != glCheckFramebufferStatus(GL_FRAMEBUFFER)) {
        std::cerr << "Could not initialize framebuffer" << std::endl;
        std::exit(EXIT_FAILURE);
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}


texture <float, cudaTextureType2D, cudaReadModeElementType> depths;

__global__
void
convert_depths(int width, int height, float znear, float zfar) {
    int const bx = blockIdx.x;
    int const tx = threadIdx.x;

    int const by = blockIdx.y;
    int const ty = threadIdx.y;

    int const x = bx * blockDim.x + tx;
    int const y = by * blockDim.y + ty;

    if (x >= width || y >= height) return;

    float depth = tex2D(depths, x, y);
}

__global__
void
raycast(cacc::Vec3f origin, cacc::Mat3f invcalib, cacc::Mat3f c2w_rot,
    cacc::BVHTree<cacc::DEVICE>::Data const bvh_tree,
    cacc::Image<float, cacc::DEVICE>::Data image)
{
    int const bx = blockIdx.x;
    int const tx = threadIdx.x;

    int const by = blockIdx.y;
    int const ty = threadIdx.y;

    int const x = bx * blockDim.x + tx;
    int const y = by * blockDim.y + ty;

    if (x >= image.width || y >= image.height) return;

    int const stride = image.pitch / sizeof(float);

    cacc::Ray ray;
    ray.origin = origin;
    cacc::Vec3f v = invcalib * cacc::Vec3f((float)x + 0.5f, (float)y + 0.5f, 1.0f);
    ray.dir = (c2w_rot * v.normalize()).normalize();
    ray.set_tmin(0.001f);
    ray.set_tmax(1000.0f);


    uint hit_face_id;
    if (cacc::tracing::trace(bvh_tree, ray, &hit_face_id)) {
        //face_id != tri_id
        cacc::Tri tri = bvh_tree.tris_ptr[hit_face_id];
        image.data_ptr[y * stride + x] = 1000.0f;
        //cacc::intersect(ray, tri, image.data_ptr + y * stride + x);
    } else {
        image.data_ptr[y * stride + x] = 0.0f;
    }
}


int main(int argc, char **argv) {
    util::system::register_segfault_handler();
    util::system::print_build_timestamp(argv[0]);

    Arguments args = parse_args(argc, argv);

    Window window("", 640, 480);

    int device;
    uint num_devices;
    CHECK(cudaGLGetDevices(&num_devices, &device, 1u, cudaGLDeviceListCurrentFrame));
    CHECK(cudaGLSetGLDevice(device));

    mve::TriangleMesh::Ptr mesh;
    try {
        mesh = mve::geom::load_ply_mesh(args.mesh);
    } catch (std::exception& e) {
        throw std::runtime_error(std::string("Could not load mesh: ") + e.what());
    }
    std::vector<uint> const & faces = mesh->get_faces();
    std::vector<math::Vec3f> const & vertices = mesh->get_vertices();
    BVHTree::Ptr bvh_tree = BVHTree::create(faces, vertices);

    ogl::ShaderProgram::Ptr sp = ogl::ShaderProgram::create();
    sp->load_vert_code(vertex_shader);
    sp->load_frag_code(fragment_shader);

    ogl::MeshRenderer::Ptr mr = ogl::MeshRenderer::create(mesh);

    int width = args.width;
    int height = args.height;
    std::string image_name = args.image_name;

    mve::Scene::Ptr scene;
    try {
        scene = mve::Scene::create(args.scene_dir);
    } catch (std::exception& e) {
        std::cerr << "Could not open scene: " << e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }

    GLuint fbo, rbo, dbo;
    setup_fbo(&fbo, &rbo, &dbo, width, height);

    ogl::Camera ogl_cam;
    float const znear = 0.1f;
    float const zfar = 1000.0f;

    mve::View::Ptr view = scene->get_view_by_id(27);

    mve::CameraInfo const & camera = view->get_camera();
    math::Vec3f origin;
    camera.fill_camera_pos(origin.begin());
    math::Matrix3f invcalib;
    camera.fill_inverse_calibration(invcalib.begin(), width, height);
    math::Matrix3f c2w_rot;
    camera.fill_cam_to_world_rot(c2w_rot.begin());

    fill_ogl_camera(camera, width, height, znear, zfar, &ogl_cam);

    std::chrono::time_point<std::chrono::system_clock> start, end;

    start = std::chrono::system_clock::now();
    glEnable(GL_DEPTH_TEST);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    glViewport(0, 0, width, height);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    sp->bind();
    sp->send_uniform("znear", znear);
    sp->send_uniform("zfar", zfar);
    sp->send_uniform("viewmat", ogl_cam.view);
    sp->send_uniform("projmat", ogl_cam.proj);
    mr->set_shader(sp);
    mr->draw();
    glFlush();
    ogl::check_gl_error();
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    end = std::chrono::system_clock::now();
    std::cout << "Rasterization: "
        << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()
        << "us" << std::endl;

    struct cudaGraphicsResource *res;
    CHECK(cudaGraphicsGLRegisterImage(&res, rbo, GL_RENDERBUFFER, cudaGraphicsRegisterFlagsNone));

    CHECK(cudaGraphicsMapResources(1, &res));
    cudaArray *array;
    CHECK(cudaGraphicsSubResourceGetMappedArray(&array, res, 0, 0));

    cudaChannelFormatDesc cdesc = cudaCreateChannelDesc(32, 0, 0, 0, cudaChannelFormatKindFloat);
    //cudaBindTextureToArray(depths, array, cdesc);
    {
        dim3 block(16, 16);
        dim3 grid((width + 15) / 16, (height + 15) / 16);
        //convert_depths<<<grid, block>>>(width, height, znear, zfar);
        CHECK(cudaDeviceSynchronize());
    }

    mve::FloatImage::Ptr depth = mve::FloatImage::create(width, height, 1);
    CHECK(cudaMemcpyFromArray(depth->begin(), array, 0, 0, width * height * 4, cudaMemcpyDeviceToHost));
    mve::image::flip<float>(depth, mve::image::FLIP_VERTICAL);
    mve::image::depthmap_convert_conventions<float>(depth, invcalib, true);

    CHECK(cudaGraphicsUnmapResources(1, &res));
    CHECK(cudaGraphicsUnregisterResource(res));

    mve::FloatImage::Ptr rdepth = mve::FloatImage::create(width, height, 1);

    start = std::chrono::system_clock::now();
    #pragma omp parallel for
    for (int y = 0; y < rdepth->height(); ++y) {
        for (int x = 0; x < rdepth->width(); ++x) {
            BVHTree::Ray ray;
            ray.origin = origin;
            math::Vec3f v = invcalib * math::Vec3f ((float)x + 0.5f, (float)y + 0.5f, 1.0f);
            ray.dir = c2w_rot.mult(v.normalized()).normalize();
            ray.tmin = 0.0f;
            ray.tmax = std::numeric_limits<float>::infinity();

            BVHTree::Hit hit;
            if (!bvh_tree->intersect(ray, &hit)) continue;

            rdepth->at(x, y, 0) = (hit.t * ray.dir).norm();
        }
    }
    end = std::chrono::system_clock::now();
    std::cout << "CPU tracing: "
        << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()
        << "us" << std::endl;
    mve::image::depthmap_convert_conventions<float>(rdepth, invcalib, false);

    cacc::select_cuda_device(3, 5);
    cacc::BVHTree<cacc::DEVICE>::Ptr dbvh_tree;
    dbvh_tree = cacc::BVHTree<cacc::DEVICE>::create<uint, math::Vec3f>(bvh_tree);
    cacc::tracing::bind_textures(dbvh_tree->cdata());
    cacc::Image<float, cacc::DEVICE>::Ptr dimage;
    dimage = cacc::Image<float, cacc::DEVICE>::create(width, height);
    cacc::Image<float, cacc::HOST>::Ptr image;
    image = cacc::Image<float, cacc::HOST>::create(width, height);
    start = std::chrono::system_clock::now();
    {
        dim3 block(16, 8);
        dim3 grid((width + 15) / 16, (height + 7) / 8);
        raycast<<<grid, block>>>(cacc::Vec3f(origin.begin()),
            cacc::Mat3f(invcalib.begin()), cacc::Mat3f(c2w_rot.begin()),
            dbvh_tree->cdata(), dimage->cdata());
        CHECK(cudaDeviceSynchronize());
    }
    end = std::chrono::system_clock::now();
    std::cout << "GPU tracing: "
        << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()
        << "us" << std::endl;
    *image = *dimage;
    CHECK(cudaDeviceSynchronize());

    mve::FloatImage::Ptr ddepth = mve::FloatImage::create(width, height, 1);
    cacc::Image<float, cacc::HOST>::Data data = image->cdata();
    for (int y = 0; y < data.height; ++y) {
        for (int x = 0; x < data.width; ++x) {
            ddepth->at(x, y, 0) = data.data_ptr[y * (data.pitch / sizeof(float)) + x];
        }
    }

    mve::FloatImage::Ptr error = mve::FloatImage::create(width, height, 1);
    for (int i = 0; i < depth->get_value_amount(); ++i) {
        error->at(i) = std::abs(ddepth->at(i) - depth->at(i));
    }

    mve::image::save_pfm_file(error, "/tmp/error.pfm");

    return EXIT_SUCCESS;
}
