#include <memory>
#include <iostream>

#include "util/system.h"
#include "util/arguments.h"
#include "util/file_system.h"

#include "util/matrix_io.h"

#include "math/matrix_tools.h"

#include "mve/image_io.h"
#include "mve/marching_cubes.h"
#include "mve/mesh_io_ply.h"

#include "ogl/events.h"
#include "ogl/camera.h"
#include "ogl/camera_trackball.h"
#include "ogl/mesh_renderer.h"

#include "sim/engine.h"
#include "sim/window.h"
#include "sim/shader.h"
#include "sim/shader_type.h"
#include "sim/entities/trajectory_renderer.h"
#include "sim/model_renderer.h"

#include "col/mpl_viridis.h"

#include "utp/bspline.h"

#include "geom/sphere.h"
#include "geom/volume_io.h"

#include "utp/trajectory_io.h"


struct Arguments {
    std::string mesh;
    std::string volume;
    std::string trajectory;
};

Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_minnum(0);
    args.set_nonopt_maxnum(0);
    args.set_usage("Usage: " + std::string(argv[0]) + " [OPTS]");
    args.add_option('m', "mesh", true, "TODO");
    args.add_option('v', "volume", true, "TODO");
    args.add_option('t', "trajectory", true, "TODO");
    args.set_description("Visualizer");
    args.parse(argc, argv);

    Arguments conf;

    for (util::ArgResult const* i = args.next_option();
         i != 0; i = args.next_option()) {
        switch (i->opt->sopt) {
        case 'm':
            conf.mesh = i->arg;
        break;
        case 'v':
            conf.volume = i->arg;
        break;
        case 't':
            conf.trajectory = i->arg;
        break;
        default:
            throw std::invalid_argument("Invalid option");
        }
    }

    return conf;
}

void
extract(math::Vector<std::uint32_t, 3> pos, Volume<std::uint32_t>::ConstPtr volume, mve::ByteImage::Ptr hist) {
    mve::FloatImage::Ptr image = volume->at(pos);

    int offset = hist->get_pixel_amount() / 2;
    static float (*colormap)[3];
    colormap = col::maps::lin::viridis;
    for (int i = 0; i < hist->get_pixel_amount() / 2; ++i) {
        float value = image ? image->at(i) : 0.0f;
        std::uint8_t lidx = std::floor(value * 255.0f);
        float t = value * 255.0f - lidx;
        std::uint8_t hidx = lidx == 255 ? 255 : lidx + 1;

        math::Vec3f lc(colormap[lidx]);
        math::Vec3f hc(colormap[hidx]);
        for (int j = 0; j < 3; ++j) {
            hist->at(offset + i, j) = 255.0f * ((1.0f - t) * lc[j] + t * hc[j]);
        }
    }
}

void
init_opengl(void) {
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_FRAMEBUFFER_SRGB);
}

int main(int argc, char **argv) {
    util::system::register_segfault_handler();
    util::system::print_build_timestamp(argv[0]);

    Arguments args = parse_args(argc, argv);

    Window window("Visualizer", 1920, 1080);
    init_opengl();

    Engine::Ptr engine(new Engine());
    if (!args.mesh.empty()) {
        engine->create_static_model(args.mesh);
    }

    std::vector<Shader::Ptr> shaders;
    if (!args.trajectory.empty()) {
        Shader::Ptr shader(new Shader());
        std::string path = util::fs::join_path(__ROOT__, "res/shaders");
        shader->load_shader_program(path + "/" + shader_names[LINES]);
        math::Matrix4f eye;
        math::matrix_set_identity(&eye);
        shader->set_model_matrix(eye);
        shaders.push_back(shader);

        mve::TriangleMesh::Ptr mesh = mve::TriangleMesh::create();
        std::vector<math::Vec3f> & verts = mesh->get_vertices();
        std::vector<uint> & faces = mesh->get_faces();
        std::vector<math::Vec4f> & colors = mesh->get_vertex_colors();

        std::vector<math::Vec3f> poss;
        float step;
        {
            std::vector<mve::CameraInfo> trajectory;
            utp::load_trajectory(args.trajectory, &trajectory);
            step = 0.1f / (trajectory.size() - 1);
            poss.reserve(trajectory.size());
            verts.reserve(trajectory.size() * 5);
            colors.reserve(trajectory.size() * 5);
            faces.reserve(trajectory.size() * 16);

            float size = 0.1f;

            for (std::size_t i = 0; i < trajectory.size(); ++i) {
                mve::CameraInfo const & cam = trajectory[i];
                math::Vec3f trans(cam.trans);
                math::Matrix3f rot(cam.rot);
                math::Vec3f pos = -rot.transposed() * trans;
                poss.push_back(pos);

                math::Vec3f rx(rot(0, 0), rot(0, 1), rot(0, 2));
                math::Vec3f ry(rot(1, 0), rot(1, 1), rot(1, 2));
                math::Vec3f rz(rot(2, 0), rot(2, 1), rot(2, 2));

                /* Derived from mve/apps/umve/scene_addins/addin_frusta_base.cc */
                std::size_t idx = verts.size();
                verts.push_back(pos);
                colors.emplace_back(0.5f, 0.5f, 0.5f, 1.0f);
                for (int j = 0; j < 4; ++j)
                {
                    math::Vec3f corner = pos + size * rz
                        + rx * size / (2.0f * cam.flen) * (j & 1 ? -1.0f : 1.0f)
                        + ry * size / (2.0f * cam.flen) * (j & 2 ? -1.0f : 1.0f);
                    verts.push_back(corner);
                    colors.emplace_back(0.5f, 0.5f, 0.5f, 1.0f);
                    faces.push_back(idx + 0); faces.push_back(idx + 1 + j);
                }
                faces.push_back(idx + 1); faces.push_back(idx + 2);
                faces.push_back(idx + 2); faces.push_back(idx + 4);
                faces.push_back(idx + 4); faces.push_back(idx + 3);
                faces.push_back(idx + 3); faces.push_back(idx + 1);
            }

            if (!faces.empty() && faces.size() % 3 == 0) {
                faces.push_back(0); faces.push_back(0); //TODO remove this hack
            }
        }

        utp::BSpline<float, 3, 3> spline;
        spline.fit(poss);

        Trajectory::Ptr trajectory(new Trajectory);
        for (float t = 0.0f; t < 1.0f; t += step) {
            trajectory->xs.push_back(spline.eval(t));
            trajectory->qs.emplace_back(math::Vec3f(0.0f, 0.0f, 1.0f), 0.0f);
        }

        Pose::Ptr pose(new Pose);
        TrajectoryRenderer::Ptr tr(new TrajectoryRenderer(trajectory, shader));
        ModelRenderer::Ptr mr(new ModelRenderer(shader));
		mr->add_mesh(mesh, nullptr);
        Entity::Ptr ret(new Entity);
        ret->add_component(pose);
        ret->add_component(trajectory);
        ret->add_component(tr);
        ret->add_component(mr);
        engine->add_entity(ret);
    }

    math::Vector<std::uint32_t, 3> pos(0, 0, 0);
    std::vector<float> samples;
    Volume<std::uint32_t>::Ptr volume;
    Pose::Ptr poses[3][3];
    mve::ByteImage::Ptr hist = mve::ByteImage::create(128, 90, 3);
    ogl::Texture::Ptr textures[3][3];
    if (!args.volume.empty()) {
        Shader::Ptr shader(new Shader());
        std::string path = util::fs::join_path(__ROOT__, "res/shaders");
        shader->load_shader_program(path + "/" + shader_names[TEXTURE]);
        math::Matrix4f eye;
        math::matrix_set_identity(&eye);
        shader->set_model_matrix(eye);
        shaders.push_back(shader);

        try {
            volume = load_volume<std::uint32_t>(args.volume);
        } catch (std::exception& e) {
            std::cerr << "Could not load volume: " << e.what() << std::endl;
            std::exit(EXIT_FAILURE);
        }

        mve::TriangleMesh::Ptr sphere = generate_sphere_mesh(0.1f, 5u);
        parameterize_spherical(sphere);

        /* Delete upper hemisphere */
        std::vector<math::Vec3f> verts = sphere->get_vertices();
        std::vector<bool> dlist(verts.size(), false);
        for (std::size_t i = 0; i < verts.size(); ++i) {
            if (verts[i][2] > 0.0f) dlist[i] = true;
        }
        sphere->delete_vertices_fix_faces(dlist);

        std::uint32_t x = volume->width() / 2;
        std::uint32_t y = volume->height() / 2;
        std::uint32_t z = volume->depth() - 2;
        pos = math::Vector<std::uint32_t, 3>(x, y, z);

        for (int ry = -1; ry <= 1; ++ry) {
            for (int rx = -1; rx <= 1; ++rx) {
                Pose::Ptr & pose = poses[ry + 1][rx + 1];
                ogl::Texture::Ptr & texture = textures[ry + 1][rx + 1];
                pose = Pose::Ptr(new Pose);
                texture = ogl::Texture::create();
                DynamicModelRenderer::Ptr mr(new DynamicModelRenderer(pose, shader));
                math::Vector<std::uint32_t, 3> rpos(x + rx, y + ry, z);
                extract(rpos, volume, hist);
                texture->upload(hist);
                pose->x = volume->position(x + rx, y + ry, z);
                pose->q = math::Quaternion<float>(math::Vec3f(0.0f, 0.0f, 1.0f), 0.0f);

                mr->add_mesh(sphere, texture);
                Entity::Ptr ret(new Entity);
                ret->add_component(pose);
                ret->add_component(mr);
                engine->add_entity(ret);
            }
        }
    }

    ogl::Camera camera;
    ogl::CamTrackball trackball;
    camera.width = 1920;
    camera.height = 1080;
    camera.right = (1920.0f / 1080.0f) * camera.top;
    trackball.set_camera(&camera);
    trackball.set_camera_params(math::Vec3f(-2.0f, 2.0f, 2.0f),
        math::Vec3f(-2.0f, 2.0f, 0.0f), math::Vec3f(0.0f, 1.0f, 0.0f));

    ogl::MouseEvent event;

    window.register_key_callback(326, [volume, &poses, hist, &textures, &pos] (int action, int) {
        if (action) {
            if (pos[0] >= volume->width() - 2) return;
            pos[0] += 1;

            for (int y = 0; y < 3; ++y) {
                std::swap(poses[y][0], poses[y][1]);
                std::swap(poses[y][1], poses[y][2]);
                std::swap(textures[y][0], textures[y][1]);
                std::swap(textures[y][1], textures[y][2]);
            }
            for (int y = 0; y < 3; ++y) {
                math::Vector<std::uint32_t, 3> rpos(pos[0] + 1, pos[1] + y - 1, pos[2]);
                poses[y][2]->x = volume->position(pos[0] + 1, pos[1] + y - 1, pos[2]);
                extract(rpos, volume, hist);
                textures[y][2]->upload(hist);
            }
        }
    });
    window.register_key_callback(324, [volume, &poses, hist, &textures, &pos] (int action, int) {
        if (action) {
            if (pos[0] <= 1) return;
            pos[0] -= 1;

            for (int y = 0; y < 3; ++y) {
                std::swap(poses[y][2], poses[y][1]);
                std::swap(poses[y][1], poses[y][0]);
                std::swap(textures[y][2], textures[y][1]);
                std::swap(textures[y][1], textures[y][0]);
            }
            for (int y = 0; y < 3; ++y) {
                math::Vector<std::uint32_t, 3> rpos(pos[0] - 1, pos[1] + y - 1, pos[2]);
                poses[y][0]->x = volume->position(pos[0] - 1, pos[1] + y - 1, pos[2]);
                extract(rpos, volume, hist);
                textures[y][0]->upload(hist);
            }
        }
    });
    window.register_key_callback(328, [volume, &poses, hist, &textures, &pos] (int action, int) {
        if (action) {
            if (pos[1] >= volume->height() - 2) return;
            pos[1] += 1;
            for (int x = 0; x < 3; ++x) {
                std::swap(poses[0][x], poses[1][x]);
                std::swap(poses[1][x], poses[2][x]);
                std::swap(textures[0][x], textures[1][x]);
                std::swap(textures[1][x], textures[2][x]);
            }
            for (int x = 0; x < 3; ++x) {
                math::Vector<std::uint32_t, 3> rpos(pos[0] + x - 1, pos[1] + 1, pos[2]);
                poses[2][x]->x = volume->position(pos[0] + x - 1, pos[1] + 1, pos[2]);
                extract(rpos, volume, hist);
                textures[2][x]->upload(hist);
            }
        }
    });
    window.register_key_callback(322, [volume, &poses, hist, &textures, &pos] (int action, int) {
        if (action) {
            if (pos[1] <= 1) return;
            pos[1] -= 1;
            for (int x = 0; x < 3; ++x) {
                std::swap(poses[2][x], poses[1][x]);
                std::swap(poses[1][x], poses[0][x]);
                std::swap(textures[2][x], textures[1][x]);
                std::swap(textures[1][x], textures[0][x]);
            }
            for (int x = 0; x < 3; ++x) {
                math::Vector<std::uint32_t, 3> rpos(pos[0] + x - 1, pos[1] - 1, pos[2]);
                poses[0][x]->x = volume->position(pos[0] + x - 1, pos[1] - 1, pos[2]);
                extract(rpos, volume, hist);
                textures[0][x]->upload(hist);
            }
        }
    });
    window.register_key_callback(329, [volume, poses, hist, textures, &pos] (int action, int) {
        if (action) {
            if (pos[2] >= volume->depth() - 2) return;
            pos[2] += 1;

            for (int y = 0; y < 3; ++y) {
                for (int x = 0; x < 3; ++x) {
                    math::Vector<std::uint32_t, 3> rpos(pos[0] + x - 1, pos[1] + y - 1, pos[2]);
                    poses[y][x]->x = volume->position(pos[0] + x - 1, pos[1] + y - 1, pos[2]);
                    extract(rpos, volume, hist);
                    textures[y][x]->upload(hist);
                }
            }
        }
    });
    window.register_key_callback(327, [volume, poses, hist, textures, &pos] (int action, int) {
        if (action) {
            if (pos[2] <= 1) return;
            pos[2] -= 1;

            for (int y = 0; y < 3; ++y) {
                for (int x = 0; x < 3; ++x) {
                    math::Vector<std::uint32_t, 3> rpos(pos[0] + x - 1, pos[1] + y - 1, pos[2]);
                    poses[y][x]->x = volume->position(pos[0] + x - 1, pos[1] + y - 1, pos[2]);
                    extract(rpos, volume, hist);
                    textures[y][x]->upload(hist);
                }
            }
        }
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

        for (std::size_t i = 0; i < shaders.size(); ++i) {
            shaders[i]->set_view_matrix(camera.view);
            shaders[i]->set_proj_matrix(camera.proj);
            shaders[i]->update();
        }

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
