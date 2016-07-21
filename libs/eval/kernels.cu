#include <cuda_runtime.h>

#include "cacc/math.h"
#include "cacc/tracing.h"
#include "cacc/nnsearch.h"

#include "kernels.h"

__forceinline__ __device__
cacc::Vec2f
project(cacc::Vec3f const & v, cacc::Mat3f const & calib)
{
    cacc::Vec3f p = calib * v;
    return cacc::Vec2f(p[0] / p[2] - 0.5f, p[1] / p[2] - 0.5f);
}

__forceinline__ __device__
cacc::Vec3f
orthogonal(cacc::Vec3f const & vec)
{
    cacc::Vec3f const n0(1.0f, 0.0f, 0.0f);
    cacc::Vec3f const n1(0.0f, 1.0f, 0.0f);
    if (abs(dot(n0, vec)) < abs(dot(n1, vec))) {
        return cross(n0, vec);
    } else {
        return cross(n1, vec);
    }
}

__forceinline__ __device__
bool
visible(cacc::Vec3f const & v, cacc::Vec3f const & v2cn, float l,
    cacc::BVHTree<cacc::DEVICE>::Data const & bvh_tree)
{
    cacc::Ray ray;
    ray.origin = v;
    ray.dir = v2cn;
    ray.set_tmin(l * 0.001f);
    ray.set_tmax(l);

    return !cacc::tracing::trace(bvh_tree, ray);
}

__forceinline__ __device__
cacc::Vec2f
relative_angle(cacc::Vec3f const & v2cn, cacc::Vec3f const & n) {
    cacc::Vec3f b0 = orthogonal(n).normalize();
    cacc::Vec3f b1 = cross(n, b0).normalize();
    cacc::Vec3f x = v2cn - dot(v2cn, n) * n;
    return cacc::Vec2f(dot(x, b0), dot(x, b1));
}

__global__
void
populate_histogram(cacc::Mat4f w2c, cacc::Mat3f calib, cacc::Vec3f view_pos,
    int width, int height,
    cacc::BVHTree<cacc::DEVICE>::Data bvh_tree,
    cacc::PointCloud<cacc::DEVICE>::Data cloud,
    cacc::VectorArray<cacc::Vec2f, cacc::DEVICE>::Data dir_hist)
{
    int const bx = blockIdx.x;
    int const tx = threadIdx.x;

    uint id = bx * blockDim.x + tx;
    int const stride = dir_hist.pitch / sizeof(cacc::Vec2f);

    if (id >= cloud.num_vertices) return;
    cacc::Vec3f v = cloud.vertices_ptr[id];
    cacc::Vec3f n = cloud.normals_ptr[id];
    cacc::Vec3f v2c = view_pos - v;
    float l = norm(v2c);
    cacc::Vec3f v2cn = v2c / l;

    float ctheta = dot(v2cn, n);
    // 0.087f ~ cos(85.0f / 180.0f * pi)
    if (abs(ctheta) < 0.087f) return;

    if (l > 80.0f) return; //TODO make configurable
    cacc::Vec2f p = project(mult(w2c, v, 1.0f), calib);

    if (p[0] < 0.0f || width <= p[0] || p[1] < 0.0f || height <= p[1]) return;

    if (!visible(v, v2cn, l, bvh_tree)) return;

    uint row = dir_hist.num_rows_ptr[id];

    if (row >= dir_hist.max_rows) return;

    cacc::Vec2f dir = relative_angle(v2cn, n);

    dir_hist.num_rows_ptr[id] = row;
    dir_hist.data_ptr[row * stride + id] = dir;

    dir_hist.num_rows_ptr[id] += 1;
}

__forceinline__ __device__
cacc::Vec2f
mean(cacc::Vec2f const * values, uint stride, uint n)
{
    cacc::Vec2f mean(0.0f);
    for (uint i = 0; i < n; ++i) {
        cacc::Vec2f dir = values[i * stride];
        mean += dir;
    }
    return mean / (float)n;
}

__forceinline__ __device__
cacc::Mat2f
covariance(cacc::Vec2f const * values, uint stride, uint n,
    cacc::Vec2f const & mean)
{
    cacc::Mat2f cov(0.0f);
    for (uint i = 0; i < n; ++i) {
        cacc::Vec2f dir = values[i * stride] - mean;
        cov[0][0] += dir[0] * dir[0];
        cov[0][1] += dir[0] * dir[1];
        cov[1][0] += dir[1] * dir[0];
        cov[1][1] += dir[1] * dir[1];
    }
    return cov / (float)n;
}

__forceinline__ __device__
cacc::Vec2f
eigen_values(cacc::Mat2f const & mat)
{
    float trace = cacc::trace(mat);
    float det = cacc::det(mat);

    float tmp = sqrt((trace * trace) / 4.0f - det);

    return cacc::Vec2f(trace / 2.0f + tmp, trace / 2.0f - tmp);
}

__forceinline__ __device__
cacc::Vec2f
eigen_values(cacc::Vec2f const * values, uint stride, uint n) {
    if (n == 0) return cacc::Vec2f(0.0f, 0.0f);
    cacc::Vec2f mu = mean(values, stride, n);
    cacc::Mat2f cov = covariance(values, stride, n, mu);
    return eigen_values(cov);
}

__global__
void
evaluate_histogram(cacc::VectorArray<cacc::Vec2f, cacc::DEVICE>::Data dir_hist)
{
    int const bx = blockIdx.x;
    int const tx = threadIdx.x;

    uint id = bx * blockDim.x + tx;

    if (id >= dir_hist.num_cols) return;

    int const stride = dir_hist.pitch / sizeof(cacc::Vec2f);
    uint num_rows = dir_hist.num_rows_ptr[id];

    cacc::Vec2f * values = dir_hist.data_ptr + id;
    cacc::Vec2f eigen = eigen_values(values, stride, num_rows);

    dir_hist.data_ptr[(dir_hist.max_rows - 1) * stride + id] = eigen;
}

__global__
void populate_histogram(cacc::Vec3f view_pos,
    cacc::BVHTree<cacc::DEVICE>::Data bvh_tree,
    cacc::PointCloud<cacc::DEVICE>::Data cloud,
    cacc::KDTree<3, cacc::DEVICE>::Data kd_tree,
    cacc::VectorArray<cacc::Vec2f, cacc::DEVICE>::Data dir_hist,
    cacc::VectorArray<float, cacc::DEVICE>::Data con_hist)
{
    int const bx = blockIdx.x;
    int const tx = threadIdx.x;

    uint id = bx * blockDim.x + tx;

    if (id >= cloud.num_vertices) return;

    cacc::Vec3f v = cloud.vertices_ptr[id];
    cacc::Vec3f n = cloud.normals_ptr[id];
    cacc::Vec3f v2c = view_pos - v;
    float l = norm(v2c);
    cacc::Vec3f v2cn = v2c / l;

    float ctheta = dot(v2cn, n);
    // 0.087f ~ cos(85.0f / 180.0f * pi)
    if (abs(ctheta) < 0.087f) return;
    if (l > 80.0f) return; //TODO make configurable
    if (!visible(v, v2cn, l, bvh_tree)) return;

    cacc::Vec2f dir = relative_angle(v2cn, n);

    if (id >= dir_hist.num_cols) return;

    int const stride = dir_hist.pitch / sizeof(cacc::Vec2f);
    uint num_rows = dir_hist.num_rows_ptr[id];

    float contrib = 0.1f; //TODO determine usefull value

    if (num_rows > 2) {
        cacc::Vec2f * values = dir_hist.data_ptr + id;
        cacc::Vec2f eigen_before = eigen_values(values, stride, num_rows);
        values[num_rows * stride] = dir;
        cacc::Vec2f eigen_after = eigen_values(values, stride, num_rows + 1);
        float recon_before = max(abs(eigen_before[0]), abs(eigen_before[1]));
        float recon_after = max(abs(eigen_after[0]), abs(eigen_after[1]));
        contrib = recon_after - recon_before;
    }

    uint idx;
    float dist;
    cacc::nnsearch::find_nns<3u>(kd_tree, -v2cn, &idx, &dist, 1u);
    atomicAdd(con_hist.data_ptr + idx, contrib);
}

constexpr float pi = 3.14159265f;

__global__
void
evaluate_histogram(cacc::Mat3f calib, int width, int height,
    cacc::KDTree<3, cacc::DEVICE>::Data const kd_tree,
    cacc::VectorArray<float, cacc::DEVICE>::Data const con_hist,
    cacc::Image<float, cacc::DEVICE>::Data hist)
{
    int const bx = blockIdx.x;
    int const tx = threadIdx.x;
    int const by = blockIdx.y;
    int const ty = threadIdx.y;

    uint x = bx * blockDim.x + tx;
    uint y = by * blockDim.y + ty;

    if (x >= hist.width || y >= hist.height) return;

    float phi = (x / (float) hist.width) * 2.0f * pi;
    float theta = (y / (float) hist.height) * pi;
    float ctheta = cos(theta);
    float stheta = sin(theta);
    float cphi = cos(phi);
    float sphi = sin(phi);
    cacc::Vec3f view_dir(stheta * cphi, stheta * sphi, ctheta);
    view_dir.normalize();

    cacc::Vec3f rz = -view_dir;

    cacc::Vec3f up = cacc::Vec3f(0.0f, 0.0f, 1.0f);
    bool stable = abs(dot(up, rz)) < 0.99f;
    up = stable ? up : cacc::Vec3f(cphi, sphi, 0.0f);

    cacc::Vec3f rx = cross(up, rz).normalize();
    cacc::Vec3f ry = cross(rz, rx).normalize();

    float sum = 0.0f;

    for (uint i = 0; i < kd_tree.num_verts; ++i) {
        cacc::Vec3f dir = kd_tree.verts_ptr[i].normalize();
        if (dot(view_dir, dir) < 0.0f) continue;

        cacc::Vec3f v;
        v[0] = dot(rx, dir);
        v[1] = dot(ry, dir);
        v[2] = dot(rz, dir);

        cacc::Vec2f p = project(v, calib);
        if (p[0] < 0.0f || width <= p[0] || p[1] < 0.0f || height <= p[1]) continue;

        sum += con_hist.data_ptr[i];
    }

    int const stride = hist.pitch / sizeof(float);
    hist.data_ptr[y * stride + x] = sum;
}

__global__
void
evaluate_histogram(cacc::KDTree<3, cacc::DEVICE>::Data const kd_tree,
    cacc::Image<float, cacc::DEVICE>::Data const hist,
    cacc::VectorArray<float, cacc::DEVICE>::Data con_hist)
{
    int const bx = blockIdx.x;
    int const tx = threadIdx.x;
    int const by = blockIdx.y;
    int const ty = threadIdx.y;

    uint x = bx * blockDim.x + tx;
    uint y = by * blockDim.y + ty;

    if (x >= hist.width || y >= hist.height) return;

    int const stride = hist.pitch / sizeof(float);
    float sum = hist.data_ptr[y * stride + x];
    float phi = (x / (float) hist.width) * 2.0f * pi;
    float theta = (y / (float) hist.height) * pi;

    cacc::Vec3f view_dir(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
    view_dir.normalize();

    uint idx;
    float dist;
    cacc::nnsearch::find_nns<3u>(kd_tree, view_dir, &idx, &dist, 1u);

    uint32_t * bin = (uint32_t *) con_hist.data_ptr + con_hist.pitch / sizeof(float) + idx;
    atomicMax(bin, cacc::float_to_uint32(sum));
}
