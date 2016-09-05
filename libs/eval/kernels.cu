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
heuristic(cacc::Vec3f const * rel_dirs, uint stride, uint n)
{
    float novelty = 1.0f;
    float matchability = 0.0f;
    cacc::Vec3f new_rel_dir = rel_dirs[(n - 1) * stride];
    float quality = dot(cacc::Vec3f(0.0f, 0.0f, 1.0f), new_rel_dir);
    for (uint i = 0; i < (n - 1); ++i) {
        cacc::Vec3f rel_dir = rel_dirs[i * stride];
        float theta = acos(dot(new_rel_dir, rel_dir));
        novelty = min(novelty, sigmoid(theta, pi / 8.0f, 16.0f));
        matchability = max(matchability,
            1.0f - sigmoid(theta, pi / 4.0f, 16.0f));
    }
    return novelty * matchability * quality;
}

__global__
void
populate_histogram(cacc::Mat4f w2c, cacc::Mat3f calib, cacc::Vec3f view_pos,
    int width, int height,
    cacc::BVHTree<cacc::DEVICE>::Data bvh_tree,
    cacc::PointCloud<cacc::DEVICE>::Data cloud,
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

    float ctheta = dot(v2cn, n);
    // 0.087f ~ cos(85.0f / 180.0f * pi)
    if (ctheta < 0.087f) return;

    if (l > 80.0f) return; //TODO make configurable
    cacc::Vec2f p = project(mult(w2c, v, 1.0f), calib);

    if (p[0] < 0.0f || width <= p[0] || p[1] < 0.0f || height <= p[1]) return;

    if (!visible(v, v2cn, l, bvh_tree)) return;

    uint row = dir_hist.num_rows_ptr[id];

    if (row >= dir_hist.max_rows) return;

    cacc::Vec3f rel_dir = relative_direction(v2cn, n);

    cacc::Vec3f * rel_dirs = dir_hist.data_ptr + id;

    dir_hist.num_rows_ptr[id] = row;
    rel_dirs[row * stride] = rel_dir;

    float recon = dot(cacc::Vec3f(0.0f, 0.0f, 1.0f), rel_dir);
    if (row >= 1) {
        float recon_before = rel_dirs[(row - 1) * stride][3];
        recon = recon_before + heuristic(rel_dirs, stride, row + 1);
    }

    rel_dirs[row * stride][3] = recon;
    dir_hist.num_rows_ptr[id] += 1;
}

__global__
void
evaluate_histogram(cacc::VectorArray<cacc::Vec3f, cacc::DEVICE>::Data dir_hist)
{
    int const bx = blockIdx.x;
    int const tx = threadIdx.x;

    uint id = bx * blockDim.x + tx;

    if (id >= dir_hist.num_cols) return;

    int const stride = dir_hist.pitch / sizeof(cacc::Vec3f);
    uint num_rows = dir_hist.num_rows_ptr[id];

    cacc::Vec3f * rel_dirs = dir_hist.data_ptr + id;

    float recon = -1.0f;

    if (num_rows >= 1.0f) {
        recon = 0.0f;
        //rel_dirs[id][3] = 0.0f;
        for (int i = 1; i <= num_rows; ++i) {
            recon += heuristic(rel_dirs, stride, i);
            //rel_dirs[i * stride + id][3] = sum;
        }
    }

    dir_hist.data_ptr[(dir_hist.max_rows - 1) * stride + id][3] = recon;
}

__global__
void populate_histogram(cacc::Vec3f view_pos,
    cacc::BVHTree<cacc::DEVICE>::Data bvh_tree,
    cacc::PointCloud<cacc::DEVICE>::Data cloud,
    cacc::KDTree<3, cacc::DEVICE>::Data kd_tree,
    cacc::VectorArray<cacc::Vec3f, cacc::DEVICE>::Data dir_hist,
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

    //TODO make configurable
    if (l > 80.0f) return;
    // 0.087f ~ cos(85.0f / 180.0f * pi)
    if (ctheta < 0.087f) return;

    if (!visible(v, v2cn, l, bvh_tree)) return;

    if (id >= dir_hist.num_cols) return;

    int const stride = dir_hist.pitch / sizeof(cacc::Vec3f);
    uint num_rows = dir_hist.num_rows_ptr[id];

    cacc::Vec3f rel_dir = relative_direction(v2cn, n);

    float contrib = 0.0f;

    if (num_rows >= dir_hist.max_rows - 1) return;

    if (num_rows >= 1) {
        cacc::Vec3f * rel_dirs = dir_hist.data_ptr + id;
        rel_dirs[num_rows * stride] = rel_dir;
        contrib = heuristic(rel_dirs, stride, num_rows + 1);
    } else {
        contrib = dot(cacc::Vec3f(0.0f, 0.0f, 1.0f), rel_dir);
    }

    uint idx;
    float dist;
    cacc::nnsearch::find_nns<3u>(kd_tree, -v2cn, &idx, &dist, 1u);
    atomicAdd(con_hist.data_ptr + idx, contrib);
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

__forceinline__ __device__
float
max5(float v0, float v1, float v2, float v3, float v4) {
    return max(v0, max(max(v1, v2), max(v3, v4)));
}

__global__
void
suppress_nonmaxima(cacc::Image<float, cacc::DEVICE>::Data const hist,
    cacc::Image<float, cacc::DEVICE>::Data out_hist)
{
    int const bx = blockIdx.x;
    int const tx = threadIdx.x;

    uint x = bx * blockDim.x + tx;

    if (x >= hist.width) return;

    int const stride = hist.pitch / sizeof(float);

    __shared__ float sm[KERNEL_BLOCK_SIZE + 4];
    /* Row wise maxima (y-2, y-1, y, y+1, y+2) mod 5. */
    float m[5];
    /* Value of pixel (y, y+1, y+2). */
    float v[3];

    /* Initialize */
    m[1] = 0.0f;
    m[2] = 0.0f;
    #pragma unroll
    for (int y = 0; y < 2; y++) {
        __syncthreads();
        sm[tx] = hist.data_ptr[y * stride + (hist.width + x - 2) % hist.width];
        if (x >= blockDim.x - 4) {
            sm[tx + 4] = hist.data_ptr[y * stride + (x + 2) % hist.width];
        }
        __syncthreads();
        m[y + 3] = max5(sm[tx], sm[tx + 1], sm[tx + 2], sm[tx + 3], sm[tx + 4]);
        v[y + 1] = sm[tx + 2];
    }

    for (int y = 0; y < hist.height; ++y) {
        __syncthreads();
        if (y < hist.height - 2) {
            sm[tx] = hist.data_ptr[(y + 2) * stride + (hist.width + x - 2) % hist.width];
            if (x >= blockDim.x - 4) {
                sm[tx + 4] = hist.data_ptr[(y + 2) * stride + (x + 2) % hist.width];
            }
        } else {
            sm[tx] = 0.0f;
            if (x >= blockDim.x - 4) {
                sm[tx + 4] = 0.0f;
            }
        }
        __syncthreads();

        float tmp = max5(sm[tx], sm[tx + 1], sm[tx + 2], sm[tx + 3], sm[tx + 4]);
        /* Avoid local memory access. */
        switch (y % 5) {
            case 0: m[0] = tmp; break;
            case 1: m[1] = tmp; break;
            case 2: m[2] = tmp; break;
            case 3: m[3] = tmp; break;
            default: m[4] = tmp;
        }

        v[0] = v[1];
        v[1] = v[2];
        v[2] = sm[tx + 2];

        if (v[0] < max5(m[0], m[1], m[2], m[3], m[4])) {
            out_hist.data_ptr[y * stride + x] = 0.0f;
        } else {
            out_hist.data_ptr[y * stride + x] = v[0];
        }
    }
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

__global__
void
estimate_capture_difficulty(
    cacc::PointCloud<cacc::DEVICE>::Data const cloud,
    cacc::BVHTree<cacc::DEVICE>::Data const bvh_tree, uint mesh_size,
    cacc::KDTree<3, cacc::DEVICE>::Data const kd_tree,
    cacc::VectorArray<float, cacc::DEVICE>::Data capture_diff)
{
    int const bx = blockIdx.x;
    int const tx = threadIdx.x;

    uint id = bx * blockDim.x + tx;

    if (id >= cloud.num_vertices) return;
    cacc::Vec3f v = cloud.vertices_ptr[id];
    cacc::Vec3f n = cloud.normals_ptr[id];
    float l = 80.0f;

    float angles = 0.0f;

    for (uint i = 0; i < kd_tree.num_verts; ++i) {
        cacc::Vec3f dir = kd_tree.verts_ptr[i].normalize();
        if (dot(dir, n) < 0.0f) continue;

        cacc::Ray ray;
        ray.origin = v;
        ray.dir = dir;
        ray.set_tmin(l * 0.001f);
        ray.set_tmax(l);

        uint face_id = mesh_size;

        cacc::tracing::trace(bvh_tree, ray, &face_id);

        if (face_id >= mesh_size) {
            angles += 1.0f;
        }
    }

    /* Fraction of observable angles. */
    capture_diff.data_ptr[0] = 1.0f - min(1.0f, (float)angles / kd_tree.num_verts / 2);
}
