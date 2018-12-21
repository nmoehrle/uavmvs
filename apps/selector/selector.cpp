/*
 * Copyright (C) 2016-2018, Nils Moehrle
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include <iostream>
#include <random>

#include "util/system.h"
#include "util/arguments.h"
#include "util/file_system.h"

#include "util/matrix_io.h"

#include "math/matrix_tools.h"

#include "mve/bundle_io.h"
#include "mve/mesh_io_ply.h"

#include "ogl/events.h"
#include "ogl/camera.h"
#include "ogl/camera_trackball.h"
#include "ogl/mesh_renderer.h"

#include "sim/window.h"
#include "sim/shader.h"
#include "sim/shader_type.h"

#include "acc/primitives.h"

#include "geom/aabb.h"
#include "geom/aabb_io.h"
#include "geom/plane_estimation.h"

#include "col/gamma.h"


struct Arguments {
    std::string bundle;
    std::string out_aabb;
    std::string out_trans;
    std::string dense_cloud;
};

Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_minnum(3);
    args.set_nonopt_maxnum(3);
    args.set_usage("Usage: " + std::string(argv[0]) + " [OPTS] BUNDLE OUT_TRANSFORM OUT_AABB");
    args.set_description("Visual selector for target areas based on bundle "
            "files. Writes out the selection as axis-aligned bounding box and "
            "according transformation.");
    args.add_option('d', "dense-cloud", true, "dense cloud to display");
    args.parse(argc, argv);

    Arguments conf;
    conf.bundle = args.get_nth_nonopt(0);
    conf.out_trans  = args.get_nth_nonopt(1);
    conf.out_aabb = args.get_nth_nonopt(2);

    for (util::ArgResult const* i = args.next_option();
         i != 0; i = args.next_option()) {
        switch (i->opt->sopt) {
        case 'd':
            conf.dense_cloud = i->arg;
        break;
        default:
            throw std::invalid_argument("Invalid option");
        }
    }

    return conf;
}

void
init_opengl(void) {
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_FRAMEBUFFER_SRGB);
}

float const pi = std::acos(-1.0f);

int main(int argc, char **argv) {
    util::system::register_segfault_handler();
    util::system::print_build_timestamp(argv[0]);

    Arguments args = parse_args(argc, argv);

    mve::TriangleMesh::Ptr dense_cloud;
    if (!args.dense_cloud.empty()) {
        try {
            dense_cloud = mve::geom::load_ply_mesh(args.dense_cloud);
        } catch (std::exception& e) {
            std::cerr << "\tCould not load dense cloud: "<< e.what() << std::endl;
            std::exit(EXIT_FAILURE);
        }
        std::for_each(dense_cloud->get_vertex_colors().begin(),
            dense_cloud->get_vertex_colors().end(),
            [] (math::Vec4f & color) { col::gamma_decode_srgb(color.begin());}
        );
    }

    mve::TriangleMesh::Ptr cloud = mve::TriangleMesh::create();
    std::vector<math::Vec3f> & verts = cloud->get_vertices();
    std::vector<math::Vec4f> & colors = cloud->get_vertex_colors();
    {
        mve::Bundle::Ptr bundle;
        try {
            bundle = mve::load_mve_bundle(args.bundle);
        } catch (std::exception& e) {
            std::cerr << "\tCould not load bundle: "<< e.what() << std::endl;
            std::exit(EXIT_FAILURE);
        }

        std::vector<mve::Bundle::Feature3D> const & features = bundle->get_features();
        verts.reserve(features.size());
        colors.reserve(features.size());
        for (mve::Bundle::Feature3D const & feature : features) {
            if (feature.refs.size() < 5) continue;
            verts.emplace_back();
            std::copy(feature.pos, feature.pos + 3, verts.back().begin());
            colors.emplace_back(1.0f);
            std::copy(feature.color, feature.color + 3, colors.back().begin());
        }
    }

    acc::AABB<math::Vec3f> aabb = estimate_aabb(verts, 0.01f);

    math::Matrix4f T(0.0f);
    {
        std::cout << "Estimating ground plane... " << std::flush;
        math::Plane3f plane = estimate_ground_plane(cloud, aabb);
        math::Vec3f n2 = plane.n.normalize();
        math::Vec3f n0 = orthogonal(n2).normalize();
        math::Vec3f n1 = n2.cross(n0).normalize();

        T(0, 0) = n0[0]; T(0, 1) = n0[1]; T(0, 2) = n0[2]; T(0, 3) = 0.0f;
        T(1, 0) = n1[0]; T(1, 1) = n1[1]; T(1, 2) = n1[2]; T(1, 3) = 0.0f;
        T(2, 0) = n2[0]; T(2, 1) = n2[1]; T(2, 2) = n2[2]; T(2, 3) = -plane.d;
        T(3, 0) =  0.0f; T(3, 1) =  0.0f; T(3, 2) =  0.0f; T(3, 3) = 1.0f;
        std::cout << "done." << std::endl;
    }

    for (std::size_t i = 0; i < verts.size(); ++i) {
        verts[i] = T.mult(verts[i], 1.0f);
    }

    if (dense_cloud != nullptr) {
        std::vector<math::Vec3f> & verts = dense_cloud->get_vertices();
        for (std::size_t i = 0; i < verts.size(); ++i) {
            verts[i] = T.mult(verts[i], 1.0f);
        }
    }

    aabb = estimate_aabb(verts, 0.01f);

    math::Vec3f t = aabb.min + (aabb.max - aabb.min) / 2.0f;
    aabb.min = aabb.min - t;
    aabb.max = aabb.max - t;
    std::for_each(colors.begin(), colors.end(), [] (math::Vec4f & color) {
        col::gamma_decode_srgb(color.begin());
    });

    cloud->ensure_normals(true, true);
    Window window("Selector", 1920, 1080);
    init_opengl();

    std::vector<Shader::Ptr> shaders;

    std::array<ShaderType, 2> shader_types = {VCOLOR, VCOLOR};
    for (ShaderType shader_type : shader_types) {
        Shader::Ptr shader(new Shader());
        std::string path = util::fs::join_path(__ROOT__, "res/shaders");
        shader->load_shader_program(path + "/" + shader_names[shader_type]);
        math::Matrix4f eye;
        math::matrix_set_identity(&eye);
        shader->set_model_matrix(eye);
        shaders.push_back(shader);
    }

    ogl::MeshRenderer::Ptr cr = ogl::MeshRenderer::create();
    cr->set_mesh(cloud);
    cr->set_shader(shaders[0]->get_shader_program());
    cr->set_primitive(GL_POINTS);

    ogl::MeshRenderer::Ptr dcr;
    if (dense_cloud) {
        dcr = ogl::MeshRenderer::create();
        dcr->set_mesh(dense_cloud);
        dcr->set_shader(shaders[0]->get_shader_program());
        dcr->set_primitive(GL_POINTS);
    }

    ogl::MeshRenderer::Ptr mr = ogl::MeshRenderer::create();
    mr->set_mesh(generate_aabb_mesh(aabb.min, aabb.max));
    mr->set_shader(shaders[1]->get_shader_program());

    ogl::Camera camera;
    ogl::CamTrackball trackball;
    camera.width = 1920;
    camera.height = 1080;
    camera.right = (1920.0f / 1080.0f) * camera.top;
    trackball.set_camera(&camera);
    trackball.set_camera_params(math::Vec3f(-2.0f, 2.0f, 2.0f),
        math::Vec3f(-2.0f, 2.0f, 0.0f), math::Vec3f(0.0f, 1.0f, 0.0f));

    ogl::MouseEvent event;

    bool update = true;
    bool render_dense = false;

    float angle = 0.0f;
    math::Vec3f scales(1.0f);

    window.register_key_callback(324, [&] (int action, int mods) {
        if (action) {
            if (mods == 1) scales[0] *= 0.99f;
            else t[0] += scales[0] * (aabb.max[0] - aabb.min[0]) * 0.01f;
            update = true;
        }
    });
    window.register_key_callback(326, [&] (int action, int mods) {
        if (action) {
            if (mods == 1) scales[0] *= 1.01f;
            else t[0] -= scales[0] * (aabb.max[0] - aabb.min[0]) * 0.01f;
            update = true;
        }
    });
    window.register_key_callback(322, [&] (int action, int mods) {
        if (action) {
            if (mods == 1) scales[1] *= 0.99f;
            else t[1] -= scales[1] * (aabb.max[1] - aabb.min[1]) * 0.01f;
            update = true;
        }
    });
    window.register_key_callback(328, [&] (int action, int mods) {
        if (action) {
            if (mods == 1) scales[1] *= 1.01f;
            else t[1] += scales[1] * (aabb.max[1] - aabb.min[1]) * 0.01f;
            update = true;
        }
    });
    window.register_key_callback(321, [&] (int action, int mods) {
        if (action) {
            if (mods == 1) scales[2] *= 0.99f;
            else t[2] -= scales[2] * (aabb.max[2] - aabb.min[2]) * 0.01f;
            update = true;
        }
    });
    window.register_key_callback(323, [&] (int action, int mods) {
        if (action) {
            if (mods == 1) scales[2] *= 1.01f;
            else t[2] += scales[2] * (aabb.max[2] - aabb.min[2]) * 0.01f;
            update = true;
        }
    });
    window.register_key_callback(327, [&] (int action, int) {
        if (action) {
            angle += pi / 180.0f;
            update = true;
        }
    });
    window.register_key_callback(329, [&] (int action, int) {
        if (action) {
            angle -= pi / 180.0f;
            update = true;
        }
    });
    window.register_key_callback(325, [&] (int action, int) {
        if (action) {
            render_dense = !render_dense;
            update = true;
        }
    });

    window.register_mouse_button_callback(0, [&] (int action) {
        event.button = ogl::MOUSE_BUTTON_LEFT;
        if (action) {
            event.type = ogl::MOUSE_EVENT_PRESS;
            event.button_mask |= ogl::MOUSE_BUTTON_LEFT;
            update = true;
        } else {
            event.type = ogl::MOUSE_EVENT_RELEASE;
            event.button_mask &= ~ogl::MOUSE_BUTTON_LEFT;
        }
        trackball.consume_event(event);
    });
    window.register_mouse_button_callback(1, [&] (int action) {
        event.button = ogl::MOUSE_BUTTON_RIGHT;
        if (action) {
            event.type = ogl::MOUSE_EVENT_PRESS;
            event.button_mask |= ogl::MOUSE_BUTTON_RIGHT;
            update = true;
        } else {
            event.type = ogl::MOUSE_EVENT_RELEASE;
            event.button_mask &= ~ogl::MOUSE_BUTTON_RIGHT;
        }
        trackball.consume_event(event);
    });
    window.register_cursor_position_callback([&] (double xpos, double ypos) {
        event.type = ogl::MOUSE_EVENT_MOVE;
        event.button = ogl::MOUSE_BUTTON_NONE;
        event.x = xpos;
        event.y = ypos;
        trackball.consume_event(event);
        update = true;
    });
    window.register_scroll_callback([&] (double, double yoffset) {
        ogl::MouseEvent event;
        if (yoffset > 0.0f) {
            event.type = ogl::MOUSE_EVENT_WHEEL_UP;
        } else {
            event.type = ogl::MOUSE_EVENT_WHEEL_DOWN;
        }
        trackball.consume_event(event);
        update = true;
    });

    while (window.good())
    {
        window.update();
        if (update) {
            camera.pos = trackball.get_campos();
            camera.viewing_dir = trackball.get_viewdir();
            camera.up_vec = trackball.get_upvec();
            camera.update_matrices();

            math::Vec3f zaxis(0.0f, 0.0f, 1.0f);
            math::Matrix3f R = matrix_rotation_from_axis_angle(zaxis, angle);
            math::Matrix3f S = matrix_from_diagonal(scales);
            math::Matrix4f T = (R * S).hstack(t).vstack(math::Vec4f(0.0f, 0.0f, 0.0f, 1.0f));
            shaders[1]->set_model_matrix(T);

            for (std::size_t i = 0; i < shaders.size(); ++i) {
                shaders[i]->set_view_matrix(camera.view);
                shaders[i]->set_proj_matrix(camera.proj);
                shaders[i]->update();
            }

            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            glViewport(0, 0, 1920, 1080);

            cr->draw();
            if (dcr && render_dense) dcr->draw();
            glEnable(GL_BLEND);
            mr->draw();
            glDisable(GL_BLEND);

            glFlush();

            update = false;
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    math::Matrix3f S = matrix_from_diagonal(scales);
    aabb.min = S * aabb.min;
    aabb.max = S * aabb.max;
    save_aabb_to_file(aabb, args.out_aabb);

    math::Vec3f zaxis(0.0f, 0.0f, 1.0f);
    math::Matrix3f R = matrix_rotation_from_axis_angle(zaxis, angle);
    T = R.transpose().hstack(-t).vstack(math::Vec4f(0.0f, 0.0f, 0.0f, 1.0f)) * T;
    save_matrix_to_file(T, args.out_trans);

    return EXIT_SUCCESS;
}
