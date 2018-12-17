/*
 * Copyright (C) 2016-2018, Nils Moehrle
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include <iostream>
#include <cassert>
#include <algorithm>

#include <fmt/format.h>

#include "util/system.h"
#include "util/arguments.h"

#include "mve/mesh_io_ply.h"
#include "mve/image_io.h"
#include "mve/mesh_tools.h"

#include "fssr/iso_octree.h"
#include "fssr/iso_surface.h"
#include "fssr/mesh_clean.h"

#include "acc/kd_tree.h"
#include "acc/primitives.h"

constexpr float lowest = std::numeric_limits<float>::lowest();

struct Arguments {
    std::string cloud;
    std::string mesh;
    std::string hmap;
    std::string scloud;
    float resolution;
    float min_distance;
};

Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_minnum(2);
    args.set_nonopt_maxnum(2);
    args.set_usage("Usage: " + std::string(argv[0]) + " [OPTS] CLOUD OUT_MESH");
    args.set_description("Generates a proxy geometry of the scene by interperting "
        "the point cloud as height map and extracting a 2.5D surface from it.\n"
        "WARNING: Assumes that the z axis corresponds to height.");
    args.add_option('r', "resolution", true, "height map resolution [-1.0]");
    args.add_option('h', "height-map", true, "save height map as pfm file");
    args.add_option('s', "sample-cloud", true, "save sample mesh as ply file");
    args.add_option('m', "min-distance", true, "minimum distance from original samples [0.0]");
    args.parse(argc, argv);

    Arguments conf;
    conf.cloud = args.get_nth_nonopt(0);
    conf.mesh = args.get_nth_nonopt(1);
    conf.resolution = -1.0f;
    conf.min_distance = 0.0f;

    for (util::ArgResult const* i = args.next_option();
         i != 0; i = args.next_option()) {
        switch (i->opt->sopt) {
        case 'r':
            conf.resolution = i->get_arg<float>();
        break;
        case 'm':
            conf.min_distance = i->get_arg<float>();
        break;
        case 'h':
            conf.hmap = i->arg;
        break;
        case 's':
            conf.scloud = i->arg;
        break;
        default:
            throw std::invalid_argument("Invalid option");
        }
    }

    if (conf.min_distance < 0.0f) {
        throw std::invalid_argument("Minimum distance may not be negative.");
    }

    return conf;
}

template <int N> inline void
patch(mve::FloatImage::Ptr img, int x, int y, float (*ptr)[N][N]) {
    static_assert(N % 2 == 1, "Requires odd patch size");
    constexpr int e = N / 2;
    for (int i = -e; i <= e; ++i) {
        for (int j = -e; j <= e; ++j) {
            (*ptr)[e + j][e + i] = img->at(x + j, y + i, 0);
        }
    }
}

template <int N>
void filter_median(mve::FloatImage::Ptr hmap, float boundary) {
    constexpr int n = N * N;

    int width = hmap->width();
    int height = hmap->height();
    mve::FloatImage::Ptr tmp = mve::FloatImage::create(width, height, 1);

    #pragma omp parallel for
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (y < (N / 2) || y >= height - (N / 2) || x < (N / 2) || x >= width - (N / 2)) {
                tmp->at(x, y, 0) = boundary;
            } else {
                float heights[n];
                patch(hmap, x, y, (float (*)[N][N])&heights);
                std::nth_element(heights, heights + (n / 2), heights + n);
                tmp->at(x, y, 0) = heights[n / 2];
            }
        }
    }

    //std::swap(*hmap, *tmp);
    hmap->swap(*tmp);
}

template <int N>
void filter_nth_lowest(mve::FloatImage::Ptr hmap, int idx, float boundary) {
    constexpr int n = N * N;
    assert(idx < n);

    int width = hmap->width();
    int height = hmap->height();
    mve::FloatImage::Ptr tmp = mve::FloatImage::create(width, height, 1);

    #pragma omp parallel for
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (y < (N / 2) || y >= height - (N / 2) || x < (N / 2) || x >= width - (N / 2)) {
                tmp->at(x, y, 0) = boundary;
            } else {
                float heights[n];
                patch(hmap, x, y, (float (*)[N][N])&heights);
                std::nth_element(heights, heights + idx, heights + n);
                tmp->at(x, y, 0) = heights[idx];
            }
        }
    }

    //std::swap(*hmap, *tmp);
    hmap->swap(*tmp);
}

float
median_distance_of_nth_nn(std::vector<math::Vec3f> const & verts,
    acc::KDTree<3, unsigned> const & kd_tree, std::size_t n)
{
    assert(verts.size() > n);

    std::vector<float> dists(verts.size());
    #pragma omp parallel for
    for (std::size_t i = 0; i < verts.size(); ++i) {
        std::vector<std::pair<unsigned, float> > nns;
        kd_tree.find_nns(verts[i], n, &nns);
        dists[i] = nns[n - 1].second;
    }

    //std::ofstream out("/tmp/dists");
    //std::copy(dists.begin(), dists.end(), std::ostream_iterator<float>(out, "\n"));
    //out.close();

    std::vector<float>::iterator nth = dists.begin() + dists.size() / 2;
    std::nth_element(dists.begin(), nth, dists.end());
    return *nth;
}

int main(int argc, char **argv) {
    util::system::register_segfault_handler();
    util::system::print_build_timestamp(argv[0]);

    Arguments args = parse_args(argc, argv);

    mve::TriangleMesh::Ptr cloud;
    try {
        cloud = mve::geom::load_ply_mesh(args.cloud);
    } catch (std::exception& e) {
        std::cerr << "\tCould not load cloud: "<< e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }

    assert(cloud->get_faces().empty());

    std::vector<math::Vec3f> const & verts = cloud->get_vertices();
    std::vector<math::Vec4f> const & colors = cloud->get_vertex_colors();
    std::vector<float> const & confs = cloud->get_vertex_confidences();

    acc::AABB<math::Vec3f> aabb = acc::calculate_aabb(verts);

    assert(acc::valid(aabb) && acc::volume(aabb) > 0.0f);

    acc::KDTree<3, unsigned> kd_tree(verts);
    if (args.resolution <= 0.0f) {
        float density;
        if (cloud->has_vertex_values()) {
            std::vector<float> scales = cloud->get_vertex_values();
            std::vector<float>::iterator nth = scales.begin() + scales.size() / 2;
            std::nth_element(scales.begin(), nth, scales.end());
            /* Assuming that the scale is the radius of a 5x5 mvs patch. */
            density = (*nth / 2.5f);
        } else {
            std::cout << "Estimating point cloud density... " << std::flush;
            density = median_distance_of_nth_nn(verts, kd_tree, 5);
            std::cout << "done." << std::endl;
        }
        args.resolution = 2.0f * density;
    }

    int width = (aabb.max[0] - aabb.min[0]) / args.resolution + 1.0f;
    int height = (aabb.max[1] - aabb.min[1]) / args.resolution + 1.0f;

    std::cout << fmt::format("Creating height map ({}x{})", width, height) << std::endl;

    bool check_confs = cloud->has_vertex_confidences() && args.min_distance == 0.0f;

    /* Create height map. */
    mve::FloatImage::Ptr hmap = mve::FloatImage::create(width, height, 1);
    hmap->fill(lowest);
    for (std::size_t i = 0; i < verts.size(); ++i) {
        if (check_confs && confs[i] == 0.0f) continue;

        math::Vec3f vertex = verts[i];
        int x = (vertex[0] - aabb.min[0]) / args.resolution;
        assert(0 <= x && x < width);
        int y = (vertex[1] - aabb.min[1]) / args.resolution;
        assert(0 <= y && y < height);
        float height = vertex[2];
        float z = hmap->at(x, y, 0);
        if (z > height) continue;

        hmap->at(x, y, 0) = height;
    }

    if (args.min_distance == 0.0f) {
        /* Use median filter to eliminate outliers. */
        filter_median<3>(hmap, lowest);
    }

    /* Fill holes within the height map. */
    bool holes = true;
    int hole_filling_iters = 20;
    while (holes && 0 < hole_filling_iters--) {
        holes = false;

        mve::FloatImage::Ptr tmp = mve::FloatImage::create(width, height, 1);

        #pragma omp parallel for schedule(dynamic)
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                if (y == 0 || y == height - 1 || x == 0 || x == width - 1) {
                    tmp->at(x, y, 0) = lowest;
                } else if (hmap->at(x, y, 0) != lowest) {
                    tmp->at(x, y, 0) = hmap->at(x, y, 0);
                } else {
                    float heights[9];
                    patch(hmap, x, y, (float (*)[3][3])&heights);

                    float * end = std::remove(heights, heights + 9, lowest);

                    int n = std::distance(heights, end);

                    if (n >= 3) {
                        std::sort(heights, end);
                        tmp->at(x, y, 0) = heights[n / 2];
                    } else {
                        tmp->at(x, y, 0) = lowest;
                        holes = true;
                    }
                }
            }
        }

        //std::swap(*hmap, *tmp);
        hmap->swap(*tmp);
    }

    /* Estimate ground level and normalize height map */
    float ground_level = std::numeric_limits<float>::max();
    #pragma omp parallel for reduction(min:ground_level)
    for (int i = 0; i < hmap->get_value_amount(); ++i) {
        float height = hmap->at(i);
        if (height != lowest && height < ground_level) {
            ground_level = height;
        }
    }

    std::size_t num_valid = 0;
    #pragma omp parallel for reduction(+:num_valid)
    for (int i = 0; i < hmap->get_value_amount(); ++i) {
        float height = hmap->at(i);
        if (height == lowest) continue;

        num_valid += 1;
        hmap->at(i) = height - ground_level;
    }

    if (args.min_distance > 0.0f) {
        /* Filter height map to ensure minimal distance. */
        float radius = args.min_distance;
        int kernel_size = std::ceil(2.0f * (radius / args.resolution) + 1);

        mve::FloatImage::Ptr kernel = mve::FloatImage::create(kernel_size, kernel_size, 1);
        for (int y = 0; y < kernel_size; ++y) {
            for (int x = 0; x < kernel_size; ++x) {
                float cx = (x - kernel_size / 2) * args.resolution;
                float cy = (y - kernel_size / 2) * args.resolution;
                float cz2 = radius * radius - (cx * cx + cy * cy) ;
                if (cz2 > 0.0f) {
                    kernel->at(x, y, 0) = std::sqrt(cz2);
                } else {
                    kernel->at(x, y, 0) = lowest;
                }
            }
        }

        mve::FloatImage::Ptr tmp = mve::FloatImage::create(width, height, 1);

        #pragma omp parallel for
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                float max = lowest;

                for (int ky = 0; ky < kernel_size; ++ky) {
                    for (int kx = 0; kx < kernel_size; ++kx) {
                        int cx = x + kx - kernel_size / 2;
                        int cy = y + ky - kernel_size / 2;
                        if (cx < 0 || width <= cx || cy < 0 || height <= cy) continue;

                        float h = hmap->at(cx, cy, 0);

                        /* Skip patches that have invalid pixels. */
                        if (h == lowest) {
                            max = lowest;
                            goto end;
                        }

                        float v = kernel->at(kx, ky, 0) + h;
                        max = std::max(max, v);
                    }
                }

                end: tmp->at(x, y, 0) = max;
            }
        }

        hmap.swap(tmp);
    }

    if (!args.hmap.empty()) {
        mve::image::save_pfm_file(hmap, args.hmap);
    }

    fssr::IsoOctree octree;

    mve::TriangleMesh::Ptr scloud = mve::TriangleMesh::create();
    std::vector<math::Vec3f> & sverts = scloud->get_vertices();
    std::vector<math::Vec3f> & snormals = scloud->get_vertex_normals();

    /* Introduce artificial samples at height discontinuities. */
    #pragma omp parallel for schedule(dynamic)
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (y <= 1 || y >= height - 2 || x <= 1 || x >= width - 2) continue;

            float heights[3][3];
            patch(hmap, x, y, &heights);

            /* Only sample valid patches. */
            {
                float * it = (float *)heights;
                bool invalid = std::any_of(it, it + 9, [](float h) {
                    return h == lowest;
                });
                if (invalid) continue;
            }

            float gx =
                (heights[0][0] - heights[2][0])
                + 2.0f * (heights[0][1] - heights[2][1])
                + (heights[0][2] - heights[2][2]);
            float gy =
                (heights[0][0] - heights[0][2])
                + 2.0f * (heights[1][0] - heights[1][2])
                + (heights[2][0] - heights[2][2]);

            float px = x * args.resolution + args.resolution / 2 + aabb.min[0];
            float py = y * args.resolution + args.resolution / 2 + aabb.min[1];

            #pragma omp critical
            {

                math::Vec3f normal(
                    gx / (8.0f * args.resolution),
                    gy / (8.0f * args.resolution),
                    1.0f);
                normal.normalize();
                sverts.emplace_back(px, py, heights[1][1] + ground_level);
                snormals.push_back(normal);
            }

            float rdx = -heights[0][1] + heights[1][1];
            float rdy = -heights[1][0] + heights[1][1];
            float fdx = -heights[1][1] + heights[2][1];
            float fdy = -heights[1][1] + heights[1][2];

            if (fdx > 0.0f && rdx < 0.0f && fdy > 0.0f && rdy < 0.0f) continue;

            /* Calculate relevant magnitude. */
            float m = std::max(std::max(rdx, -fdx), std::max(rdy, -fdy));
            math::Vec3f normal(gx, gy, 0.0f);
            normal.normalize();

            if (m / args.resolution < 1.5f) continue;

            for (int i = 1; i < m / args.resolution; ++i) {
                float pz = ground_level + heights[1][1] - i * args.resolution;
                math::Vec3f vertex(px, py, pz);

                #pragma omp critical
                {
                    sverts.push_back(vertex);
                    snormals.push_back(normal);
                }
            }
        }
    }

    if (!args.scloud.empty()) {
        mve::geom::SavePLYOptions opts;
        opts.write_vertex_normals = true;
        mve::geom::save_ply_mesh(scloud, args.scloud, opts);
    }

    for (std::size_t i = 0; i < sverts.size(); ++i) {
        fssr::Sample sample;
        sample.pos = sverts[i];
        sample.normal = snormals[i];
        /* Set scale according to fssr scale (radius of patch). */
        sample.scale = args.resolution * 1.25f;
        sample.confidence = 1.0f;
        std::vector<std::pair<uint, float> > nns;
        kd_tree.find_nns(sverts[i], 3, &nns, args.resolution);
        if (!nns.empty()) {
            math::Vec3f color(0.0f);
            float norm = 0.0f;
            for (std::size_t n = 0; n < nns.size(); ++n) {
                float weight = 1.0f - nns[n].second / args.resolution;
                color += weight * math::Vec3f(colors[nns[n].first].begin());
                norm += weight;
            }
            sample.color = color / norm;
        } else {
            sample.color = math::Vec3f(0.415f, 0.353f, 0.80f);
        }
        octree.insert_sample(sample);
    }

    /* Perform fssrecon c.f. mve/apps/fssrecon/fssrecon.cc */
    octree.limit_octree_level();
    octree.compute_voxels();
    octree.clear_samples();
    fssr::IsoSurface iso_surface(&octree, fssr::INTERPOLATION_CUBIC);
    mve::TriangleMesh::Ptr mesh = iso_surface.extract_mesh();

    {
        std::size_t num_vertices = mesh->get_vertices().size();
        std::vector<float> confidences = mesh->get_vertex_confidences();
        mve::TriangleMesh::DeleteList delete_verts(num_vertices, false);
        for (std::size_t i = 0; i < num_vertices; ++i) {
            if (confidences[i] == 0.0f) {
                delete_verts[i] = true;
            }
        }
        mesh->delete_vertices_fix_faces(delete_verts);
    }

    mve::geom::mesh_components(mesh, num_valid);
    fssr::clean_mc_mesh(mesh);

    mve::geom::SavePLYOptions opts;
    opts.write_vertex_normals = true;
    mve::geom::save_ply_mesh(mesh, args.mesh, opts);

    return EXIT_SUCCESS;
}
