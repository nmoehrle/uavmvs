#include <fstream>

#include "mve/scene.h"
#include "mve/camera.h"
#include "mve/mesh_io_ply.h"

#include "acc/kd_tree.h"
#include "acc/bvh_tree.h"

#include "cacc/point_cloud.h"

void load_scene_as_trajectory(std::string const & path, std::vector<mve::CameraInfo> * trajectory) {
    mve::Scene::Ptr scene;
    try {
        scene = mve::Scene::create(path);
    } catch (std::exception& e) {
        std::cerr << "Could not open scene: " << e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }

    for (mve::View::Ptr const & view : scene->get_views()) {
        if (view == nullptr) continue;
        trajectory->push_back(view->get_camera());
    }
}

void load_trajectory(std::string const & path,
    std::vector<mve::CameraInfo> * trajectory)
{
    std::ifstream in(path.c_str());
    if (!in.good()) throw std::runtime_error("Could not open trajectory file");
    std::size_t length;
    in >> length;

    trajectory->resize(length);

    for (std::size_t i = 0; i < length; ++i) {
        math::Vec3f pos;
        for (int j = 0; j < 3; ++j) {
            in >> pos[j];
        }
        math::Matrix3f rot;
        for (int j = 0; j < 9; ++j) {
            in >> rot[j];
        }
        math::Vec3f trans = -rot * pos;

        mve::CameraInfo & cam = trajectory->at(i);
        cam.flen = 0.86f; //TODO save and read from file
        std::copy(trans.begin(), trans.end(), cam.trans);
        std::copy(rot.begin(), rot.end(), cam.rot);
    }

    if (in.fail()) {
        in.close();
        throw std::runtime_error("Invalid trajectory file");
    }

    in.close();
}

void save_trajectory(std::vector<mve::CameraInfo> const & trajectory,
    std::string const & path)
{
    std::ofstream out(path.c_str());
    if (!out.good()) throw std::runtime_error("Could not open trajectory file for writing");
    std::size_t length = trajectory.size();
    out << length << std::endl;

    for (std::size_t i = 0; i < length; ++i) {
        mve::CameraInfo const & cam = trajectory[i];
        math::Vec3f trans(cam.trans);
        math::Matrix3f rot(cam.rot);
        math::Vec3f pos = -rot.transposed() * trans;

        out << pos << std::endl;
        out << rot;
    }

    out.close();
}

acc::KDTree<3, uint>::Ptr
load_mesh_as_kd_tree(std::string const & path)
{
    mve::TriangleMesh::Ptr mesh;
    try {
        mesh = mve::geom::load_ply_mesh(path);
    } catch (std::exception& e) {
        std::cerr << "\tCould not load mesh: "<< e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }

    std::vector<math::Vec3f> const & vertices = mesh->get_vertices();
    return acc::KDTree<3, uint>::create(vertices);
}

acc::BVHTree<uint, math::Vec3f>::Ptr
load_mesh_as_bvh_tree(std::string const & path)
{
    mve::TriangleMesh::Ptr mesh;
    try {
        mesh = mve::geom::load_ply_mesh(path);
    } catch (std::exception& e) {
        std::cerr << "\tCould not load mesh: "<< e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }

    std::vector<math::Vec3f> const & vertices = mesh->get_vertices();
    std::vector<uint> const & faces = mesh->get_faces();
    return acc::BVHTree<uint, math::Vec3f>::create(faces, vertices);
}

cacc::PointCloud<cacc::HOST>::Ptr
load_point_cloud(std::string const & path)
{
    mve::TriangleMesh::Ptr mesh;
    try {
        mesh = mve::geom::load_ply_mesh(path);
    } catch (std::exception& e) {
        std::cerr << "\tCould not load mesh: " << e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }
    mesh->ensure_normals(true, true);

    std::vector<math::Vec3f> const & vertices = mesh->get_vertices();
    std::vector<math::Vec3f> const & normals = mesh->get_vertex_normals();

    cacc::PointCloud<cacc::HOST>::Ptr ret;
    ret = cacc::PointCloud<cacc::HOST>::create(vertices.size());
    cacc::PointCloud<cacc::HOST>::Data data = ret->cdata();
    for (std::size_t i = 0; i < vertices.size(); ++i) {
        data.vertices_ptr[i] = cacc::Vec3f(vertices[i].begin());
        data.normals_ptr[i] = cacc::Vec3f(normals[i].begin());
    }

    if (mesh->has_vertex_values()) {
        std::vector<float> const & values = mesh->get_vertex_values();
        for (std::size_t i = 0; i < values.size(); ++i) {
            data.values_ptr[i] = values[i];
        }
    }

    return ret;
}

