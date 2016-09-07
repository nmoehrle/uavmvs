#ifndef KERNELS_HEADER
#define KERNELS_HEADER

#include "cacc/image.h"
#include "cacc/matrix.h"
#include "cacc/kd_tree.h"
#include "cacc/bvh_tree.h"
#include "cacc/point_cloud.h"
#include "cacc/vector_array.h"

#define KERNEL_BLOCK_SIZE 128
__global__
void populate_histogram(cacc::Vec3f view_pos, float max_distance,
    cacc::Mat4f w2c, cacc::Mat3f calib, int width, int height,
    cacc::BVHTree<cacc::DEVICE>::Data bvh_tree,
    cacc::PointCloud<cacc::DEVICE>::Data cloud,
    cacc::VectorArray<cacc::Vec3f, cacc::DEVICE>::Data hist);

__global__
void evaluate_histogram(
    cacc::VectorArray<cacc::Vec3f, cacc::DEVICE>::Data hist);

__global__
void populate_histogram(cacc::Vec3f view_pos, float max_distance,
    cacc::BVHTree<cacc::DEVICE>::Data bvh_tree,
    cacc::PointCloud<cacc::DEVICE>::Data cloud,
    cacc::KDTree<3, cacc::DEVICE>::Data kd_tree,
    cacc::VectorArray<cacc::Vec3f, cacc::DEVICE>::Data dir_hist,
    cacc::VectorArray<float, cacc::DEVICE>::Data con_hist);

__global__
void evaluate_histogram(cacc::Image<float, cacc::DEVICE>::Data hist);

__global__
void suppress_nonmaxima(cacc::Image<float, cacc::DEVICE>::Data const hist,
    cacc::Image<float, cacc::DEVICE>::Data out_hist);

__global__
void initialize_histogram(cacc::VectorArray<float, cacc::DEVICE>::Data con_hist);

__global__
void evaluate_histogram(cacc::Mat3f calib, int width, int height,
    cacc::KDTree<3, cacc::DEVICE>::Data const kd_tree,
    cacc::VectorArray<float, cacc::DEVICE>::Data const con_hist,
    cacc::Image<float, cacc::DEVICE>::Data hist);

__global__
void evaluate_histogram(cacc::KDTree<3, cacc::DEVICE>::Data const kd_tree,
    cacc::Image<float, cacc::DEVICE>::Data const hist,
    cacc::VectorArray<float, cacc::DEVICE>::Data con_hist);

__global__
void estimate_capture_difficulty(float max_distance,
    cacc::BVHTree<cacc::DEVICE>::Data const bvh_tree, uint mesh_size,
    cacc::KDTree<3, cacc::DEVICE>::Data const kd_tree,
    cacc::PointCloud<cacc::DEVICE>::Data const cloud);

//TODO Homogenize naming scheme (vertices vs. verts)
//TODO Introduce eval namespace
//TODO Actually create a lib

#endif /* KERNELS_HEADER */
