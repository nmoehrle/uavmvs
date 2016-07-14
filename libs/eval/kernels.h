#ifndef KERNELS_HEADER
#define KERNELS_HEADER

#include "cacc/image.h"
#include "cacc/matrix.h"
#include "cacc/bvh_tree.h"
#include "cacc/point_cloud.h"
#include "cacc/vector_array.h"

#define KERNEL_BLOCK_SIZE 128
__global__
void populate_histogram(cacc::Mat4f w2c, cacc::Mat3f calib,
    cacc::Vec3f view_pos, int width, int height,
    cacc::BVHTree<cacc::DEVICE>::Data bvh_tree,
    cacc::PointCloud<cacc::DEVICE>::Data cloud,
    cacc::VectorArray<cacc::DEVICE, cacc::Vec2f>::Data hist);

__global__
void evaluate_histogram(
    cacc::VectorArray<cacc::DEVICE, cacc::Vec2f>::Data hist);

__global__
void populate_histogram(cacc::Vec3f view_pos,
    cacc::BVHTree<cacc::DEVICE>::Data bvh_tree,
    cacc::PointCloud<cacc::DEVICE>::Data cloud,
    cacc::Image<float, cacc::DEVICE>::Data hist);

__global__
void evaluate_histogram(
    cacc::Image<float, cacc::DEVICE>::Data hist);

//TODO Homogenize template order (cacc::VectorArray<cacc::Vec2f, cacc::Device>)
//TODO Introduce eval namespace

#endif /* KERNELS_HEADER */
