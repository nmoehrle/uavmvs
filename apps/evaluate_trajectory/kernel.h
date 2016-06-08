#pragma once
#include "cacc/bvh_tree.h"
#include "cacc/point_cloud.h"
#include "cacc/vector_array.h"
#include "cacc/matrix.h"

#define KERNEL_BLOCK_SIZE 128
__global__
void kernel(cacc::Mat4f w2c, cacc::Mat3f calib, cacc::Vec3f view_pos,
    int width, int height,
    cacc::BVHTree<cacc::DEVICE>::Data bvh_tree,
    cacc::PointCloud<cacc::DEVICE>::Data cloud,
    cacc::VectorArray<cacc::DEVICE, cacc::Vec2f>::Data dir_hist);
