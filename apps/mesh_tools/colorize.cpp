/*
 * Copyright (C) 2016-2018, Nils Moehrle
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include <iostream>

#include "util/arguments.h"
#include "mve/mesh_io_ply.h"

#include "col/mpl_viridis.h"
#include "col/dcm_coolwarm.h"
#include "util/choices.h"

enum ColorMap {
    VIRIDIS = 0,
    COOLWARM = 1
};

template <> inline
const std::vector<std::string> choice_strings<ColorMap>() {
    return {"viridis", "coolwarm"};
}

math::Vec4f parse_color(std::string const & str) {
    unsigned num;
    std::stringstream ss(str);
    ss >> std::hex >> num;
    float r = (num >>  0 & 0xFF) / 255.0f;
    float g = (num >>  8 & 0xFF) / 255.0f;
    float b = (num >> 16 & 0xFF) / 255.0f;
    float a = (num >> 24 & 0xFF) / 255.0f;
    return math::Vec4f(r, g, b, a);
}

struct Arguments {
    std::string in_mesh;
    std::string out_mesh;
    ColorMap colormap;
    math::Vec4f ccolor;
};

Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_maxnum(2);
    args.set_nonopt_minnum(2);
    args.set_usage("Usage: " + std::string(argv[0]) + " [OPTS] IN_MESH OUT_MESH");
    args.set_description("Maps the per vertex value to color "
        "values out of range [0:1] will be mapped to ccolor");
    args.add_option('c', "ccolor", true, "colorvalue for out of range values [FF00FFFF]");
    args.add_option('m', "colormap", true, "colorize according to given colormap "
        + choices<ColorMap>(VIRIDIS));
    args.parse(argc, argv);

    Arguments conf;
    conf.in_mesh = args.get_nth_nonopt(0);
    conf.out_mesh = args.get_nth_nonopt(1);
    conf.colormap = VIRIDIS;
    conf.ccolor = math::Vec4f(1.0f, 0.0f, 1.0f, 1.0f);

    for (util::ArgResult const* i = args.next_option();
         i != 0; i = args.next_option()) {
        switch (i->opt->sopt) {
        case 'c':
            conf.ccolor = parse_color(i->arg);
        break;
        case 'm':
            conf.colormap = parse_choice<ColorMap>(i->arg);
        break;
        default:
            throw std::invalid_argument("Invalid option");
        }
    }

    return conf;
}

int main(int argc, char **argv) {
    Arguments args = parse_args(argc, argv);

    mve::TriangleMesh::Ptr mesh;
    try {
        mesh = mve::geom::load_ply_mesh(args.in_mesh);
    } catch (std::exception& e) {
        std::cerr << "Could not load mesh: " << e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }

    mve::TriangleMesh::VertexList const & vertices = mesh->get_vertices();
    mve::TriangleMesh::ColorList & colors = mesh->get_vertex_colors();
    mve::TriangleMesh::ValueList & values = mesh->get_vertex_values();
    colors.resize(vertices.size());

    if (!mesh->has_vertex_values()) {
        std::cerr << "Mesh does't contain vertex values!"<< std::endl;
        std::exit(EXIT_FAILURE);
    }

    float (*colormap)[3];
    switch(args.colormap) {
        case VIRIDIS: colormap = col::maps::srgb::viridis; break;
        case COOLWARM: colormap = col::maps::coolwarm; break;
        default: colormap = col::maps::srgb::viridis;
    }

    for (std::size_t i = 0; i < vertices.size(); i++){
        float value = values[i];

        math::Vec4f color(args.ccolor);
        if (0.0f <= value && value <= 1.0f) {
            std::uint8_t lidx = std::floor(value * 255.0f);
            float t = value * 255.0f - lidx;
            std::uint8_t hidx = lidx == 255 ? 255 : lidx + 1;

            math::Vec3f lc(colormap[lidx]);
            math::Vec3f hc(colormap[hidx]);
            for (int j = 0; j < 3; ++j) {
                color[j] = (1.0f - t) * lc[j] + t * hc[j];
            }
        }
        colors[i] = color;
    }

    mve::geom::SavePLYOptions opts;
    opts.write_vertex_normals = true;
    opts.write_vertex_colors = true;
    opts.write_vertex_values = true;

    mve::geom::save_ply_mesh(mesh, args.out_mesh, opts);
}
