/*
 * Copyright (C) 2016-2018, Nils Moehrle
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#ifndef KERNELS_HEADER
#define KERNELS_HEADER

#include "cacc/image.h"
#include "cacc/matrix.h"
#include "cacc/array.h"
#include "cacc/kd_tree.h"
#include "cacc/bvh_tree.h"
#include "cacc/point_cloud.h"
#include "cacc/vector_array.h"

#define KERNEL_BLOCK_SIZE 128

/* Add (populate) observation rays for each sample visible in the view.
 * If populate == false marks rays invalid instead
 * WARNING process_observation_rays has to be called prior to
 * evaluate_observation_rays when populate == false. */
__global__ void update_observation_rays(bool populate,
    cacc::Vec3f view_pos, float max_distance, cacc::Mat4f w2c,
    cacc::Mat3f calib, int width, int height,
    cacc::BVHTree<cacc::DEVICE>::Accessor const bvh_tree,
    cacc::PointCloud<cacc::DEVICE>::Data const cloud,
    cacc::VectorArray<cacc::Vec3f, cacc::DEVICE>::Data obs_rays);

/* Sort and remove invalid observation rays. */
__global__ void process_observation_rays(
    cacc::VectorArray<cacc::Vec3f, cacc::DEVICE>::Data obs_rays);

/* Evaluate per sample reconstructabilities based on their observation rays. */
__global__ void evaluate_observation_rays(
    cacc::VectorArray<cacc::Vec3f, cacc::DEVICE>::Data obs_rays,
    cacc::Array<float, cacc::DEVICE>::Data recons);

__global__ void populate_spherical_histogram(cacc::Vec3f view_pos, float max_distance,
    cacc::BVHTree<cacc::DEVICE>::Accessor const bvh_tree,
    cacc::PointCloud<cacc::DEVICE>::Data const cloud,
    cacc::KDTree<3, cacc::DEVICE>::Accessor const kd_tree,
    cacc::Array<float, cacc::DEVICE>::Data obs_rays);

/* Populate spherical histogram by calculating the contribution to each visible
 * cloud vertex and adding them for each bin.
 * bvh_tree - scene approximation mesh
 * cloud - samples of the scene approximation
 * kd_tree - vertices of a sphere (to index into the sphererical histogram)
 * obs_rays - observation rays for each sample
 * recons - current reconstructabilities for each sample */
__global__ void populate_spherical_histogram(cacc::Vec3f view_pos,
    float max_distance, float target_recon,
    cacc::BVHTree<cacc::DEVICE>::Accessor const bvh_tree,
    cacc::PointCloud<cacc::DEVICE>::Data const cloud,
    cacc::KDTree<3, cacc::DEVICE>::Accessor const kd_tree,
    cacc::VectorArray<cacc::Vec3f, cacc::DEVICE>::Data obs_rays,
    cacc::Array<float, cacc::DEVICE>::Data recons,
    cacc::Array<float, cacc::DEVICE>::Data sphere_hist);

/* Evaluates the spherical histogram for viewing directions of the lower
 * hemisphere by "convolving" it with a "frustum kernel".
 * Each x of hist is phi [0, width] -> [0, 2pi] and
 * y of hist is phi [0, height] -> [pi/2, pi].
 * kd_tree - vertices are 3D location of sphere_hist bins */
__global__
void evaluate_spherical_histogram(cacc::Mat3f calib, int width, int height,
    cacc::KDTree<3, cacc::DEVICE>::Accessor const kd_tree,
    cacc::Array<float, cacc::DEVICE>::Data const sphere_hist,
    cacc::Image<float, cacc::DEVICE>::Data hist);

/* Estimate the capture difficulty of each cloud vertex by sampling which parts
 * of the hemisphere around the samples normal are observable.
 * bvh_tree - contains both proxy and airspace mesh, the face IDs of the
 * airspace mesh have to start at mesh_size
 * kd_tree - vertices of a sphere */
__global__
void estimate_capture_difficulty(float max_distance,
    cacc::BVHTree<cacc::DEVICE>::Accessor const bvh_tree, uint mesh_size,
    cacc::KDTree<3, cacc::DEVICE>::Accessor const kd_tree,
    cacc::PointCloud<cacc::DEVICE>::Data const cloud);

/* Calculate per sample part of objective function. */
__global__
void calculate_func_recons(
    cacc::Array<float, cacc::DEVICE>::Data recons,
    float traget_recon,
    cacc::Array<float, cacc::DEVICE>::Data wrecons);

void configure_heuristic(float m_k, float m_x0, float t_k, float t_x0);

//TODO Introduce namespace

#endif /* KERNELS_HEADER */
