#ifndef KERNELS_HEADER
#define KERNELS_HEADER

#include "cacc/image.h"
#include "cacc/matrix.h"
#include "cacc/array.h"
#include "cacc/kd_tree.h"
#include "cacc/bvh_tree.h"
#include "cacc/point_cloud.h"
#include "cacc/vector_array.h"

#define RECONSTRUCTABLE 3.0f

#define KERNEL_BLOCK_SIZE 128
__global__
void update_direction_histogram(bool populate,
    cacc::Vec3f view_pos, float max_distance,
    cacc::Mat4f w2c, cacc::Mat3f calib, int width, int height,
    cacc::BVHTree<cacc::DEVICE>::Accessor const bvh_tree,
    cacc::PointCloud<cacc::DEVICE>::Data const cloud,
    cacc::VectorArray<cacc::Vec3f, cacc::DEVICE>::Data dir_hist);

__global__
void process_direction_histogram(
    cacc::VectorArray<cacc::Vec3f, cacc::DEVICE>::Data dir_hist);

__global__
void evaluate_direction_histogram(
    cacc::VectorArray<cacc::Vec3f, cacc::DEVICE>::Data dir_hist,
    cacc::Array<float, cacc::DEVICE>::Data recons);

__global__
void populate_histogram(cacc::Vec3f view_pos, float max_distance,
    cacc::BVHTree<cacc::DEVICE>::Accessor const bvh_tree,
    cacc::PointCloud<cacc::DEVICE>::Data const cloud,
    cacc::KDTree<3, cacc::DEVICE>::Data const kd_tree,
    cacc::VectorArray<float, cacc::DEVICE>::Data obs_hist);

__global__
void populate_histogram(cacc::Vec3f view_pos, float max_distance, float avg_recon,
    cacc::BVHTree<cacc::DEVICE>::Accessor const bvh_tree,
    cacc::PointCloud<cacc::DEVICE>::Data const cloud,
    cacc::KDTree<3, cacc::DEVICE>::Data const kd_tree,
    cacc::VectorArray<cacc::Vec3f, cacc::DEVICE>::Data dir_hist,
    cacc::Array<float, cacc::DEVICE>::Data recons,
    cacc::VectorArray<float, cacc::DEVICE>::Data con_hist);

__global__
void evaluate_histogram(cacc::Image<float, cacc::DEVICE>::Data hist);

__global__
void initialize_histogram(cacc::VectorArray<float, cacc::DEVICE>::Data con_hist);

__global__
void evaluate_histogram(cacc::Mat3f calib, int width, int height,
    cacc::KDTree<3, cacc::DEVICE>::Data const kd_tree,
    cacc::VectorArray<float, cacc::DEVICE>::Data const con_hist,
    cacc::Image<float, cacc::DEVICE>::Data hist);

__global__
void estimate_capture_difficulty(float max_distance,
    cacc::BVHTree<cacc::DEVICE>::Accessor const bvh_tree, uint mesh_size,
    cacc::KDTree<3, cacc::DEVICE>::Data const kd_tree,
    cacc::PointCloud<cacc::DEVICE>::Data const cloud);

//TODO Homogenize naming scheme (vertices vs. verts)
//TODO Introduce namespace

#endif /* KERNELS_HEADER */
