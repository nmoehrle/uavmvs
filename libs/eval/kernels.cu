#include <cuda_runtime.h>

#include "cacc/math.h"

#define TRACING_SSTACK_SIZE 12
#include "cacc/tracing.h"
#define NNSEARCH_SSTACK_SIZE 12
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
cacc::Vec3f
relative_direction(cacc::Vec3f const & v2cn, cacc::Vec3f const & n) {
    cacc::Vec3f rx = orthogonal(n).normalize();
    cacc::Vec3f ry = cross(n, rx).normalize();
    cacc::Vec3f rz = n;

    cacc::Vec3f rel_dir;
    rel_dir[0] = dot(rx, v2cn);
    rel_dir[1] = dot(ry, v2cn);
    rel_dir[2] = dot(rz, v2cn);

    return rel_dir;
}

constexpr float pi = 3.14159265f;

__forceinline__ __device__
float
sigmoid(float x, float x0, float k) {
    return 1.0f / (1.0f + __expf(-k * (x - x0)));
}

__forceinline__ __device__
float
heuristic(cacc::Vec3f const * rel_dirs, uint stride, uint n, cacc::Vec3f new_rel_dir)
{
    float theta = pi;
    for (uint i = 0; i < n; ++i) {
        cacc::Vec3f rel_dir = rel_dirs[i * stride];
        theta = min(theta, acosf(dot(new_rel_dir, rel_dir)));
    }
    float quality = dot(cacc::Vec3f(0.0f, 0.0f, 1.0f), new_rel_dir);
    float novelty = sigmoid(theta, pi / 8.0f, 16.0f);
    float matchability = 1.0f - sigmoid(theta, pi / 4.0f, 16.0f);
    return novelty * matchability * quality;
}

__global__
void
populate_direction_histogram(cacc::Vec3f view_pos, float max_distance,
    cacc::Mat4f w2c, cacc::Mat3f calib, int width, int height,
    cacc::BVHTree<cacc::DEVICE>::Data bvh_tree,
    cacc::PointCloud<cacc::DEVICE>::Data cloud,
    cacc::Array<float, cacc::DEVICE>::Data recons,
    cacc::VectorArray<cacc::Vec3f, cacc::DEVICE>::Data dir_hist)
{
    int const bx = blockIdx.x;
    int const tx = threadIdx.x;

    uint id = bx * blockDim.x + tx;
    int const stride = dir_hist.pitch / sizeof(cacc::Vec3f);

    if (id >= cloud.num_vertices) return;
    cacc::Vec3f v = cloud.vertices_ptr[id];
    cacc::Vec3f n = cloud.normals_ptr[id];
    cacc::Vec3f v2c = view_pos - v;
    float l = norm(v2c);
    cacc::Vec3f v2cn = v2c / l;

    float cphi = dot(v2cn, n);
    // 0.087f ~ cos(85.0f / 180.0f * pi)
    if (cphi < 0.087f) return;

    if (l > max_distance) return;
    cacc::Vec2f p = project(mult(w2c, v, 1.0f), calib);

    if (p[0] < 0.0f || width <= p[0] || p[1] < 0.0f || height <= p[1]) return;

    if (!visible(v, v2cn, l, bvh_tree)) return;

    uint num_rows = dir_hist.num_rows_ptr[id];

    if (num_rows >= dir_hist.max_rows) return;

    cacc::Vec3f rel_dir = relative_direction(v2cn, n);
    cacc::Vec3f * rel_dirs = dir_hist.data_ptr + id;
    rel_dirs[num_rows * stride] = rel_dir;
    dir_hist.num_rows_ptr[id] += 1;

    float recon = 0.0f;
    if (num_rows >= 1) {
        float recon_before = recons.data_ptr[id];
        recon = recon_before + heuristic(rel_dirs, stride, num_rows, rel_dir);
    }

    //recons.data_ptr[id] = recon;
    recons.data_ptr[id] = min(recon, RECONSTRUCTABLE);
}

__global__
void
evaluate_direction_histogram(
    cacc::VectorArray<cacc::Vec3f, cacc::DEVICE>::Data dir_hist,
    cacc::Array<float, cacc::DEVICE>::Data recons)
{
    int const bx = blockIdx.x;
    int const tx = threadIdx.x;

    uint id = bx * blockDim.x + tx;

    if (id >= dir_hist.num_cols) return;

    int const stride = dir_hist.pitch / sizeof(cacc::Vec3f);
    uint num_rows = dir_hist.num_rows_ptr[id];

    cacc::Vec3f * rel_dirs = dir_hist.data_ptr + id;

    float recon = 0.0f;
    if (num_rows > 1) {
        for (int i = 1; i < num_rows; ++i) {
            cacc::Vec3f rel_dir = rel_dirs[i * stride];
            recon += heuristic(rel_dirs, stride, i, rel_dir);
        }
    }

    //recons.data_ptr[id] = recon;
    recons.data_ptr[id] = min(recon, RECONSTRUCTABLE);
}

__global__
void populate_histogram(cacc::Vec3f view_pos, float max_distance,
    cacc::BVHTree<cacc::DEVICE>::Data bvh_tree,
    cacc::PointCloud<cacc::DEVICE>::Data cloud,
    cacc::KDTree<3, cacc::DEVICE>::Data kd_tree,
    cacc::VectorArray<float, cacc::DEVICE>::Data obs_hist)
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

    float scale = 1.0f - l / max_distance;
    if (scale <= 0.0f) return;

    float cphi = dot(v2cn, n);
    // 0.087f ~ cos(85.0f / 180.0f * pi)
    if (cphi <= 0.087f) return;

    float capture_difficulty = cloud.values_ptr[id];
    if (capture_difficulty <= 0.0f) return;

    if (!visible(v, v2cn, l, bvh_tree)) return;


    float score = scale * capture_difficulty * cphi;

    uint idx;
    float dist;
    cacc::nnsearch::find_nns<3u>(kd_tree, -v2cn, &idx, &dist, 1u);
    atomicAdd(con_hist.data_ptr + idx, score);
}

__global__
void populate_histogram(cacc::Vec3f view_pos, float max_distance,
    cacc::BVHTree<cacc::DEVICE>::Data bvh_tree,
    cacc::PointCloud<cacc::DEVICE>::Data cloud,
    cacc::KDTree<3, cacc::DEVICE>::Data kd_tree,
    cacc::VectorArray<cacc::Vec3f, cacc::DEVICE>::Data dir_hist,
    cacc::Array<float, cacc::DEVICE>::Data recons,
    cacc::VectorArray<float, cacc::DEVICE>::Data con_hist)
{
    int const bx = blockIdx.x;
    int const tx = threadIdx.x;

    uint id = bx * blockDim.x + tx;

    if (id >= cloud.num_vertices) return;

    if (recons.data_ptr[id] >= RECONSTRUCTABLE) return;

    cacc::Vec3f v = cloud.vertices_ptr[id];
    cacc::Vec3f n = cloud.normals_ptr[id];
    cacc::Vec3f v2c = view_pos - v;
    float l = norm(v2c);
    cacc::Vec3f v2cn = v2c / l;

    float scale = 1.0f - l / max_distance;
    if (scale <= 0.0f) return;

    float cphi = dot(v2cn, n);
    // 0.087f ~ cos(85.0f / 180.0f * pi)
    if (cphi < 0.087f) return;

    float capture_difficulty = cloud.values_ptr[id];
    if (capture_difficulty <= 0.0f) return;

    if (!visible(v, v2cn, l, bvh_tree)) return;
    assert(id < dir_hist.num_cols);

    int const stride = dir_hist.pitch / sizeof(cacc::Vec3f);
    uint num_rows = dir_hist.num_rows_ptr[id];

    if (num_rows >= dir_hist.max_rows) return;

    float contrib = cphi;
    cacc::Vec3f rel_dir = relative_direction(v2cn, n);
    if (num_rows >= 1) {
        cacc::Vec3f * rel_dirs = dir_hist.data_ptr + id;
        contrib = heuristic(rel_dirs, stride, num_rows, rel_dir);
    }

    contrib = scale * capture_difficulty * contrib;

    uint idx;
    float dist;
    cacc::nnsearch::find_nns<3u>(kd_tree, -v2cn, &idx, &dist, 1u);
    atomicAdd(con_hist.data_ptr + idx, contrib * capture_difficulty);
}

__global__
void
initialize_histogram(cacc::VectorArray<float, cacc::DEVICE>::Data con_hist)
{
    int const bx = blockIdx.x;
    int const tx = threadIdx.x;

    uint id = bx * blockDim.x + tx;

    if (id >= con_hist.num_cols) return;

    int const stride = con_hist.pitch / sizeof(float);
    con_hist.data_ptr[id] = 0.0f;
    con_hist.data_ptr[id + stride] = cacc::float_to_uint32(0.0f);
}

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

    float theta = (x / (float) hist.width) * 2.0f * pi;
    //float theta = (y / (float) hist.height) * pi;
    float phi = (0.5f + (y / (float) hist.height) / 2.0f) * pi;
    float sphi = sinf(phi);
    cacc::Vec3f view_dir(cosf(theta) * sphi, sinf(theta) * sphi, cosf(phi));
    view_dir.normalize();

    cacc::Vec3f rz = -view_dir;

    cacc::Vec3f up = cacc::Vec3f(0.0f, 0.0f, 1.0f);
    bool stable = abs(dot(up, rz)) < 0.99f;
    up = stable ? up : cacc::Vec3f(cosf(theta), sinf(theta), 0.0f);

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

__global__ void
estimate_capture_difficulty(float max_distance,
    cacc::BVHTree<cacc::DEVICE>::Data const bvh_tree, uint mesh_size,
    cacc::KDTree<3, cacc::DEVICE>::Data const kd_tree,
    cacc::PointCloud<cacc::DEVICE>::Data const cloud)
{
    int const bx = blockIdx.x;
    int const tx = threadIdx.x;

    uint id = bx * blockDim.x + tx;
    if (id >= cloud.num_vertices) return;

    cacc::Vec3f v = cloud.vertices_ptr[id];
    cacc::Vec3f n = cloud.normals_ptr[id];
    float l = max_distance;

    float sum = 0.0f;

    for (uint i = 0; i < kd_tree.num_verts; ++i) {
        cacc::Vec3f dir = kd_tree.verts_ptr[i].normalize();
        float cphi = dot(dir, n);
        if (cphi < 0.087f) continue;

        cacc::Ray ray;
        ray.origin = v;
        ray.dir = dir;
        ray.set_tmin(l * 0.001f);
        ray.set_tmax(l);

        uint face_id;
        if (!cacc::tracing::trace(bvh_tree, ray, &face_id) || face_id >= mesh_size) continue;

        cacc::Vec3f rel_dir = relative_direction(dir, n);
        float observation_angle = dot(cacc::Vec3f(0.0f, 0.0f, 1.0f), rel_dir);

        sum += observation_angle;
    }

    /* Num samples on hemisphere times expectation of sample (derivation in the thesis) */
    float max = (kd_tree.num_verts / 2.0f) * 0.5f;

    cloud.values_ptr[id] = sum / max;
}
