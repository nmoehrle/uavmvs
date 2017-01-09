#include <iostream>

#include "util/system.h"
#include "util/arguments.h"
#include "util/file_system.h"

#include "mve/mesh_io_ply.h"

#include "cacc/util.h"
#include "cacc/math.h"
#include "cacc/kd_tree.h"
#include "cacc/bvh_tree.h"
#include "cacc/point_cloud.h"
#include "cacc/vector_array.h"

#include "util/io.h"

#include "geom/sphere.h"

#include "eval/kernels.h"

struct Arguments {
    std::string cloud;
    std::string proxy_mesh;
    std::string airspace_mesh;
    std::string ocloud;
    float max_distance;
};

Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_minnum(4);
    args.set_nonopt_maxnum(4);
    args.set_usage("Usage: " + std::string(argv[0])
        + " [OPTS] IN_CLOUD PROXY_MESH AIRSPACE_MESH OUT_CLOUD");
    args.set_description("Estimates the difficulty to capture each vertex by "
        "sampling how much of its hemisphere is visible from the airspace");
    args.add_option('\0', "max-distance", true, "maximum distance to surface [80.0]");
    args.parse(argc, argv);

    Arguments conf;
    conf.cloud = args.get_nth_nonopt(0);
    conf.proxy_mesh = args.get_nth_nonopt(1);
    conf.airspace_mesh = args.get_nth_nonopt(2);
    conf.ocloud = args.get_nth_nonopt(3);
    conf.max_distance = 80.0f;

    for (util::ArgResult const* i = args.next_option();
         i != 0; i = args.next_option()) {
        switch (i->opt->sopt) {
        case '\0':
            if (i->opt->lopt == "max-distance") {
                conf.max_distance = i->get_arg<float>();
            } else {
                throw std::invalid_argument("Invalid option");
            }
        break;
        default:
            throw std::invalid_argument("Invalid option");
        }
    }

    return conf;
}

int main(int argc, char **argv) {
    util::system::register_segfault_handler();
    util::system::print_build_timestamp(argv[0]);

    Arguments args = parse_args(argc, argv);

    cacc::select_cuda_device(3, 5);

    mve::TriangleMesh::Ptr cloud;
    try {
        cloud = mve::geom::load_ply_mesh(args.cloud);
    } catch (std::exception& e) {
        std::cerr << "\tCould not load cloud: " << e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }
    std::vector<math::Vec3f> const & vertices = cloud->get_vertices();
    std::vector<math::Vec3f> const & normals = cloud->get_vertex_normals();

    cacc::PointCloud<cacc::HOST>::Ptr hcloud;
    cacc::PointCloud<cacc::DEVICE>::Ptr dcloud;
    {
        hcloud = cacc::PointCloud<cacc::HOST>::create(vertices.size());
        cacc::PointCloud<cacc::HOST>::Data data = hcloud->cdata();
        for (std::size_t i = 0; i < vertices.size(); ++i) {
            data.vertices_ptr[i] = cacc::Vec3f(vertices[i].begin());
            data.normals_ptr[i] = cacc::Vec3f(normals[i].begin());
        }
        dcloud = cacc::PointCloud<cacc::DEVICE>::create<cacc::HOST>(hcloud);
    }

    uint num_faces = 0;

    cacc::BVHTree<cacc::DEVICE>::Ptr dbvh_tree;
    {
        mve::TriangleMesh::Ptr mesh;
        try {
            mesh = mve::geom::load_ply_mesh(args.proxy_mesh);
        } catch (std::exception& e) {
            std::cerr << "\tCould not load mesh: " << e.what() << std::endl;
            std::exit(EXIT_FAILURE);
        }

        std::vector<math::Vec3f> vertices = mesh->get_vertices();
        std::vector<uint> faces = mesh->get_faces();

        uint num_verts = vertices.size();
        num_faces = faces.size() / 3;

        try {
            mesh = mve::geom::load_ply_mesh(args.airspace_mesh);
        } catch (std::exception& e) {
            std::cerr << "\tCould not load mesh: " << e.what() << std::endl;
            std::exit(EXIT_FAILURE);
        }

        std::vector<math::Vec3f> const & avertices = mesh->get_vertices();
        std::vector<uint> afaces = mesh->get_faces();
        std::for_each(afaces.begin(), afaces.end(), [num_verts] (uint & vid) { vid += num_verts; });

        vertices.insert(vertices.end(), avertices.begin(), avertices.end());

        faces.insert(faces.end(), afaces.begin(), afaces.end());

        acc::BVHTree<uint, math::Vec3f>::Ptr bvh_tree;
        bvh_tree = acc::BVHTree<uint, math::Vec3f>::create(faces, vertices);
        dbvh_tree = cacc::BVHTree<cacc::DEVICE>::create<uint, math::Vec3f>(bvh_tree);
    }

    cacc::KDTree<3u, cacc::DEVICE>::Ptr dkd_tree;
    {
        mve::TriangleMesh::Ptr sphere = generate_sphere_mesh(1.0f, 3u);
        std::vector<math::Vec3f> const & verts = sphere->get_vertices();
        acc::KDTree<3u, uint>::Ptr kd_tree = acc::KDTree<3, uint>::create(verts);
        dkd_tree = cacc::KDTree<3u, cacc::DEVICE>::create<uint>(kd_tree);
    }

    {
        dim3 grid(cacc::divup(vertices.size(), KERNEL_BLOCK_SIZE));
        dim3 block(KERNEL_BLOCK_SIZE);
        estimate_capture_difficulty<<<grid, block>>>(args.max_distance,
            dbvh_tree->accessor(), num_faces, dkd_tree->accessor(), dcloud->cdata());
        CHECK(cudaDeviceSynchronize());
    }

    std::vector<float> & values = cloud->get_vertex_values();
    values.resize(vertices.size());
    std::vector<float> & confidences = cloud->get_vertex_confidences();
    confidences.resize(vertices.size());

    *hcloud = *dcloud;
    {
        cacc::PointCloud<cacc::HOST>::Data data = hcloud->cdata();

        for (std::size_t i = 0; i < vertices.size(); ++i) {
            values[i] = data.values_ptr[i];
            confidences[i] = data.qualities_ptr[i];
        }
    }

    mve::geom::SavePLYOptions opts;
    opts.write_vertex_values = true;
    opts.write_vertex_normals = true;
    opts.write_vertex_confidences = true;
    mve::geom::save_ply_mesh(cloud, args.ocloud, opts);

    return EXIT_SUCCESS;
}
