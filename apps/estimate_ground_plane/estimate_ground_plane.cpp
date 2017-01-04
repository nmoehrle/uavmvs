#include <random>
#include <fstream>
#include <iostream>

#include "util/system.h"
#include "util/arguments.h"
#include "util/file_system.h"

#include "util/matrix_io.h"

#include "math/plane.h"
#include "math/matrix_svd.h"

#include "mve/scene.h"
#include "mve/mesh_io_ply.h"

#include "acc/primitives.h"


struct Arguments {
    std::string path;
    std::string transform;
    std::string plane;
};

Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_minnum(1);
    args.set_nonopt_maxnum(1);
    args.set_usage("Usage: " + std::string(argv[0]) + " [OPTS] SCENE/CLOUD");
    args.set_description("Estimates the ground plane.");
    args.add_option('t', "transform", true, "save transform to matrix file");
    args.add_option('p', "plane", true, "save ground plane to ply file");
    args.parse(argc, argv);

    Arguments conf;
    conf.path = args.get_nth_nonopt(0);

    for (util::ArgResult const* i = args.next_option();
         i != 0; i = args.next_option()) {
        switch (i->opt->sopt) {
        case 't':
            conf.transform = i->arg;
        break;
        case 'p':
            conf.plane = i->arg;
        break;
        default:
            throw std::invalid_argument("Invalid option");
        }
    }

    return conf;
}

math::Vec3f orthogonal(math::Vec3f const & vec) {
    math::Vec3f const n0(1.0f, 0.0f, 0.0f);
    math::Vec3f const n1(0.0f, 1.0f, 0.0f);
    if (std::abs(n0.dot(vec)) < std::abs(n1.dot(vec))) {
        return n0.cross(vec);
    } else {
        return n1.cross(vec);
    }
}

math::Plane3f
least_squares_plane(std::vector<math::Vec3f> const & vertices,
    std::vector<std::size_t> const & indices)
{
    if (indices.size() < 3) {
        throw std::invalid_argument("Expecting three or more points");
    }

    /* Calculate centroid. */
    math::Vec3d c(0.0f);
    for (std::size_t i = 0; i < indices.size(); ++i) {
        c += vertices[indices[i]];
    }
    c /= double(indices.size());

    math::Matrix3d cov(0.0);

    /* Calculate covariance matrix. */
    for (std::size_t i = 0; i < indices.size(); ++i) {
        math::Vec3f const & v = vertices[indices[i]];
        cov(0, 0) += (v[0] - c[0]) * (v[0] - c[0]);
        cov(1, 1) += (v[1] - c[1]) * (v[1] - c[1]);
        cov(2, 2) += (v[2] - c[2]) * (v[2] - c[2]);
        cov(0, 1) += (v[0] - c[0]) * (v[1] - c[1]);
        cov(0, 2) += (v[0] - c[0]) * (v[2] - c[2]);
        cov(1, 2) += (v[1] - c[1]) * (v[2] - c[2]);
    }

    cov(1, 0) = cov(0, 1);
    cov(2, 0) = cov(0, 2);
    cov(2, 1) = cov(1, 2);

    math::Matrix3d V;
    math::matrix_svd<double, 3, 3>(cov, nullptr, nullptr, &V);
    math::Vec3f normal = V.col(2);

    /* Select normal with positive z-direction. */
    if (normal[2] < 0.0)
        normal *= -1.0;

    return math::Plane3f(normal, c);
}

math::Plane3f
estimate_ground_plane(mve::TriangleMesh::ConstPtr cloud, acc::AABB<math::Vec3f> aabb) {
    float threshold = (aabb.max - aabb.min).norm() * 1e-4f;

    std::vector<math::Vec3f> const & verts = cloud->get_vertices();

    std::vector<std::size_t> inliers;

    std::default_random_engine gen;
    std::uniform_int_distribution<std::size_t> dis(0, verts.size());

    #pragma omp parallel
    {
        std::vector<std::size_t> current_inliers;
        current_inliers.reserve(verts.size());

        #pragma omp for
        for (std::size_t iteration = 0; iteration < 1000; ++iteration) {
            current_inliers.clear();

            math::Vec3f v1, v2, v3;
            #pragma omp critical
            {
                v1 = verts[dis(gen)];
                v2 = verts[dis(gen)];
                v3 = verts[dis(gen)];
            }
            math::Plane3f plane(v1, v2, v3);

            for (std::size_t i = 0; i < verts.size(); ++i) {
                if (std::abs(plane.point_dist(verts[i])) > threshold) continue;
                current_inliers.push_back(i);
            }

            #pragma omp critical
            if (current_inliers.size() > inliers.size()) {
                std::swap(current_inliers, inliers);
            }
        }
    }

    math::Plane3f plane = least_squares_plane(verts, inliers);

    uint above = 0;
    uint below = 0;
    for (std::size_t i = 0; i < verts.size(); ++i) {
        if (plane.point_dist(verts[i]) > 0.0f) {
            above += 1;
        } else {
            below += 1;
        }
    }

    if (above < below) {
        plane = plane.invert();
    }

    return plane;
}

math::Plane3f
estimate_ground_plane(mve::Scene::Ptr scene, acc::AABB<math::Vec3f> aabb) {
    std::vector<mve::View::Ptr> const & views = scene->get_views();

    std::vector<math::Vec3f> view_dirs;
    view_dirs.reserve(views.size());
    for (std::size_t i = 0; i < views.size(); ++i) {
        mve::View::Ptr const & view = views[i];
        if (view == nullptr) continue;

        math::Vec3f view_dir;
        view->get_camera().fill_viewing_direction(view_dir.begin());
        view_dirs.push_back(view_dir);
    }

    for (int j = 0; j < 3; ++j) {
        std::stable_sort(view_dirs.begin(), view_dirs.end(),
            [j] (math::Vec3f const & r, math::Vec3f const & l) {
                return r[j] > l[j];
            }
        );
    }

    math::Vec3f normal = view_dirs[view_dirs.size() / 2] * -1.0f;

    math::Vec3f point = (aabb.max + aabb.min) / 2.0f;
    math::Plane3f plane(normal, point);

    mve::Bundle::ConstPtr bundle = scene->get_bundle();
    mve::Bundle::Features const & features = bundle->get_features();

    std::vector<float> dists(features.size());
    for (std::size_t i = 0; i < features.size(); ++i) {
        dists[i] = plane.point_dist(math::Vec3f(features[i].pos));
    }

    auto nth = dists.begin() + dists.size() / 20;
    std::nth_element(dists.begin(), nth, dists.end());

    return math::Plane3f(normal, point + normal * *nth);
}

int main(int argc, char **argv) {
    util::system::register_segfault_handler();
    util::system::print_build_timestamp(argv[0]);

    Arguments args = parse_args(argc, argv);

    mve::TriangleMesh::Ptr cloud;
    math::Plane3f plane;
    acc::AABB<math::Vec3f> aabb;
    if (util::fs::file_exists(args.path.c_str())) {
        try {
            cloud = mve::geom::load_ply_mesh(args.path);
        } catch (std::exception& e) {
            std::cerr << "Could not load cloud: "<< e.what() << std::endl;
            std::exit(EXIT_FAILURE);
        }
        aabb = acc::calculate_aabb(cloud->get_vertices());
        plane = estimate_ground_plane(cloud, aabb);
    } else if (util::fs::dir_exists(args.path.c_str())) {
        mve::Scene::Ptr scene;
        try {
            scene = mve::Scene::create(args.path);
        } catch (std::exception& e) {
            std::cerr << "Could not open scene: " << e.what() << std::endl;
            std::exit(EXIT_FAILURE);
        }
        mve::Bundle::ConstPtr bundle = scene->get_bundle();
        mve::Bundle::Features const & features = bundle->get_features();

        aabb.min = math::Vec3f(std::numeric_limits<float>::max());
        aabb.max = math::Vec3f(std::numeric_limits<float>::lowest());
        for (std::size_t i = 0; i < features.size(); ++i) {
            for (int j = 0; j < 3; ++j) {
                aabb.min[j] = std::min(aabb.min[j], features[i].pos[j]);
                aabb.max[j] = std::max(aabb.max[j], features[i].pos[j]);
            }
        }

        plane = estimate_ground_plane(scene, aabb);
    } else {
        std::cerr << "Path does not exist" << std::endl;
        std::exit(EXIT_FAILURE);
    }


    math::Vec3f n2 = plane.n.normalize();
    math::Vec3f n0 = orthogonal(n2).normalize();
    math::Vec3f n1 = n2.cross(n0).normalize();

    if (!args.transform.empty()) {
        math::Matrix4f T(0.0f);
        T(0, 0) = n0[0]; T(0, 1) = n0[1]; T(0, 2) = n0[2]; T(0, 3) = 0.0f;
        T(1, 0) = n1[0]; T(1, 1) = n1[1]; T(1, 2) = n1[2]; T(1, 3) = 0.0f;
        T(2, 0) = n2[0]; T(2, 1) = n2[1]; T(2, 2) = n2[2]; T(2, 3) = -plane.d;
        T(3, 0) =  0.0f; T(3, 1) =  0.0f; T(3, 2) =  0.0f; T(3, 3) = 1.0f;
        save_matrix_to_file(T, args.transform);
    }

    if (!args.plane.empty()) {
        mve::TriangleMesh::Ptr omesh = mve::TriangleMesh::create();
        mve::TriangleMesh::VertexList & overts = omesh->get_vertices();
        mve::TriangleMesh::FaceList & ofaces = omesh->get_faces();

        math::Vec3f v = plane.n * plane.d;

        float r = (aabb.max - aabb.min).norm() / std::sqrt(3)  / 2.0f;
        overts.push_back(v + n0 *  r + n1 *  r);
        overts.push_back(v + n0 * -r + n1 *  r);
        overts.push_back(v + n0 *  r + n1 * -r);
        overts.push_back(v + n0 * -r + n1 * -r);
        std::size_t indices[] = {0, 1, 2, 2, 1, 3};
        ofaces.insert(ofaces.end(), &indices[0], &indices[6]);

        mve::geom::save_ply_mesh(omesh, args.plane);
    }

    return EXIT_SUCCESS;
}
