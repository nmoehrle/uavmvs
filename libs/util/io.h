#include <fstream>
#include <cstring>

#include "mve/scene.h"
#include "mve/camera.h"
#include "mve/mesh_io_ply.h"

#include "acc/kd_tree.h"
#include "acc/bvh_tree.h"

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

template <typename T> void
save_vector(std::vector<T> const & vector, std::string const & filename) {
    std::ofstream out(filename.c_str(), std::ios::binary);
    if (!out.good()) throw util::FileException(filename, std::strerror(errno));

    out.write(reinterpret_cast<const char*>(vector.data()), vector.size() * sizeof(T));
    out.close();
}

template <typename T> std::vector<T>
load_vector(std::string const & filename) {
    std::ifstream in(filename.c_str(), std::ios::binary);
    if (!in.good()) throw util::FileException(filename, std::strerror(errno));

    in.seekg (0, in.end);
    const size_t filesize = in.tellg();
    in.seekg (0, in.beg);
    const size_t num_elements = filesize / sizeof(T);

    std::vector<T> ret(num_elements);
    in.read(reinterpret_cast<char*>(ret.data()), num_elements * sizeof(T));
    in.close();

    return ret;
}
