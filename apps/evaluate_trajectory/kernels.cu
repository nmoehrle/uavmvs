#include "cacc/tracing.h"

#include "kernels.h"

__forceinline__ __device__
cacc::Vec2f
project(cacc::Vec3f const & v,
    cacc::Mat4f const & w2c, cacc::Mat3f const & calib)
{
    cacc::Vec3f p = calib * mult(w2c, v, 1.0f);
    return cacc::Vec2f(p[0] / p[2] - 0.5f, p[1] / p[2] - 0.5f);
}

__global__
void
populate_histogram(cacc::Mat4f w2c, cacc::Mat3f calib, cacc::Vec3f view_pos,
    int width, int height,
    cacc::BVHTree<cacc::DEVICE>::Data bvh_tree,
    cacc::PointCloud<cacc::DEVICE>::Data cloud,
    cacc::VectorArray<cacc::DEVICE, cacc::Vec2f>::Data dir_hist)
{
    int const bx = blockIdx.x;
    int const tx = threadIdx.x;

    uint id = bx * blockDim.x + tx;
    int const stride = dir_hist.pitch / sizeof(cacc::Vec2f);

    if (id >= cloud.num_vertices) return;
    cacc::Vec3f v = cloud.vertices_ptr[id];
    cacc::Vec3f v2c = view_pos - v;
    float n = norm(v2c);
    //if (n > 80.0f) return; //TODO make configurable
    cacc::Vec2f p = project(v, w2c, calib);

    if (p[0] < 0.0f || width <= p[0] || p[1] < 0.0f || height <= p[1]) return;

    cacc::Ray ray;
    ray.origin = v + v2c * 0.001f;
    ray.dir = v2c / n;
    ray.set_tmin(0.0f);
    ray.set_tmax(inf);

    uint hit_face_id;
    cacc::tracing::trace(bvh_tree, ray, &hit_face_id);
    if (hit_face_id != cacc::tracing::NAI) return;

    uint row = dir_hist.num_rows_ptr[id];

    if (row >= dir_hist.max_rows) return;

    dir_hist.num_rows_ptr[id] = row;
    cacc::Vec2f dir(atan2(ray.dir[1], ray.dir[0]), acos(ray.dir[2]));
    dir_hist.data_ptr[row * stride + id] = dir;

    dir_hist.num_rows_ptr[id] += 1;
}

__global__
void
evaluate_histogram(cacc::VectorArray<cacc::DEVICE, cacc::Vec2f>::Data dir_hist)
{
    int const bx = blockIdx.x;
    int const tx = threadIdx.x;

    uint id = bx * blockDim.x + tx;
    int const stride = dir_hist.pitch / sizeof(cacc::Vec2f);

    if (id >= dir_hist.num_cols) return;

    uint num_rows = dir_hist.num_rows_ptr[id];

    cacc::Vec2f mean(0.0f);
    for (uint row = 0; row < num_rows; ++row) {
        cacc::Vec2f dir = dir_hist.data_ptr[row * stride + id];
        mean += dir;
    }
    mean /= num_rows;

    //Sample mean/covariance?

    cacc::Mat2f cov(0.0f);
    for (uint row = 0; row < num_rows; ++row) {
        cacc::Vec2f dir = dir_hist.data_ptr[row * stride + id] - mean;
        cov[0][0] += dir[0] * dir[0];
        cov[0][1] += dir[0] * dir[1];
        cov[1][0] += dir[1] * dir[0];
        cov[1][1] += dir[1] * dir[1];
    }
    cov /= num_rows;

    float trace = cacc::trace(cov);
    float det = cacc::det(cov);

    float tmp = sqrt((trace * trace) / 4.0f - det);

    cacc::Vec2f eigen(trace / 2.0f + tmp, trace / 2.0f - tmp);
    dir_hist.data_ptr[(dir_hist.max_rows - 1) * stride + id] = eigen;
}
