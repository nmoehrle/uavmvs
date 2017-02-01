#include <cuda_runtime.h>

#include "cacc/math.h"

#define TRACING_SSTACK_SIZE 8
#include "cacc/tracing.h"
#include "cacc/nnsearch.h"

#include "kernels.h"

#define SQR(x) ((x) * (x))

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
    cacc::BVHTree<cacc::DEVICE>::Accessor const & bvh_tree)
{
#if UNSAFE
    cacc::Ray ray;
    ray.origin = v + v2cn * l;
    ray.dir = -v2cn;
    ray.set_tmin(0.0f);
    ray.set_tmax(l * 0.999f);
#else
    cacc::Ray ray;
    ray.origin = v;
    ray.dir = v2cn;
    ray.set_tmin(l * 0.001f);
    ray.set_tmax(l);
#endif
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

    return rel_dir.normalize();
}

constexpr float pi = 3.14159265f;

__forceinline__ __device__
float
logistic(float x, float k, float x0) {
    return 1.0f / (1.0f + __expf(-k * (x - x0)));
}


__forceinline__ __device__
float
func(float recon, float target_recon) {
    return __powf(abs(min(recon - target_recon, 0.0f)), 2.0f);
}

__constant__ float sym_m_k = 8.0f;
__constant__ float sym_m_x0 = 4.0f;
__constant__ float sym_t_k = 32.0f;
__constant__ float sym_t_x0 = 16.0f;

void configure_heuristic(float m_k, float m_x0, float t_k, float t_x0) {
    CHECK(cudaMemcpyToSymbol(sym_m_k, &m_k, sizeof(float)));
    CHECK(cudaMemcpyToSymbol(sym_m_x0, &m_x0, sizeof(float)));
    CHECK(cudaMemcpyToSymbol(sym_t_k, &t_k, sizeof(float)));
    CHECK(cudaMemcpyToSymbol(sym_t_x0, &t_x0, sizeof(float)));
};

__forceinline__ __device__
float
rad2deg(float rad) {
    return (rad / pi) * 180.0f;
}

__forceinline__ __device__
float
heuristic(cacc::Vec3f const * rel_rays, uint stride, uint n, cacc::Vec3f new_rel_ray)
{
    float sum = 0.0f;
    for (uint i = 0; i < n; ++i) {
        cacc::Vec3f rel_ray = rel_rays[i * stride];

        float calpha = dot(new_rel_ray, rel_ray);
        float alpha = acosf(max(-1.0f, min(calpha, 1.0f)));

        float scale = min(rel_ray[3], new_rel_ray[3]);
        float ctheta = min(rel_ray[2], new_rel_ray[2]);
#if 1
        float matchability = (1.0f - logistic(alpha, sym_m_k, pi / sym_m_x0)) * ctheta;
        float triangulation = logistic(alpha, sym_t_k, pi / sym_t_x0) * scale;
        sum += matchability * triangulation;
#else
        float deg = rad2deg(alpha);
        float sigma2 = (deg < 20.0f) ? SQR(5.0f) : SQR(15.0f);
        float trimatch = exp(- SQR(deg - 20.0f) / (2.0f * sigma2));
        sum += trimatch * scale * ctheta;
#endif
    }
    return sum;
}

__global__
void
update_observation_rays(bool populate,
    cacc::Vec3f view_pos, float max_distance,
    cacc::Mat4f w2c, cacc::Mat3f calib, int width, int height,
    cacc::BVHTree<cacc::DEVICE>::Accessor const bvh_tree,
    cacc::PointCloud<cacc::DEVICE>::Data cloud,
    cacc::VectorArray<cacc::Vec3f, cacc::DEVICE>::Data obs_rays)
{
    int const bx = blockIdx.x;
    int const tx = threadIdx.x;

    uint id = bx * blockDim.x + tx;
    int const stride = obs_rays.pitch / sizeof(cacc::Vec3f);

    if (id >= cloud.num_vertices) return;
    cacc::Vec3f v = cloud.vertices_ptr[id];
    cacc::Vec3f n = cloud.normals_ptr[id];
    cacc::Vec3f v2c = view_pos - v;
    float l = norm(v2c);
    cacc::Vec3f v2cn = v2c / l;

    float ctheta = dot(v2cn, n);
    // 0.087f ~ cos(85.0f / 180.0f * pi)
    if (ctheta < 0.087f) return;

    if (l >= max_distance) return;
    cacc::Vec2f p = project(mult(w2c, v, 1.0f), calib);

    if (p[0] < 0.0f || width <= p[0] || p[1] < 0.0f || height <= p[1]) return;

    if (!visible(v, v2cn, l, bvh_tree)) return;

    cacc::Vec3f rel_ray = relative_direction(v2cn, n);
    float scale = 1.0f - (l / max_distance);
    rel_ray[3] = scale;

    if (populate) {
        uint num_rows = atomicAdd(obs_rays.num_rows_ptr + id, 1u);

        if (num_rows >= obs_rays.max_rows) return; //TODO handle overflow

        cacc::Vec3f * rel_rays = obs_rays.data_ptr + id;
        rel_rays[num_rows * stride] = rel_ray;
    } else {
        uint num_rows = min(obs_rays.num_rows_ptr[id], obs_rays.max_rows);

        cacc::Vec3f * rel_rays = obs_rays.data_ptr + id;
        for (int i = 0; i < num_rows; ++i) {
            cacc::Vec3f orel_ray = rel_rays[i * stride];

            bool equal = true;
            #pragma unroll
            for (int j = 0; j < 4; ++j) {
                equal = equal && abs(rel_ray[j] - orel_ray[j]) < 1e-5f;
            }

            if (equal) {
                //Mark invalid
                rel_rays[i * stride][3] = -1.0f;
                return;
            }
        }
    }
}

__global__
void
process_observation_rays(
    cacc::VectorArray<cacc::Vec3f, cacc::DEVICE>::Data obs_rays)
{
    int const bx = blockIdx.x;
    int const tx = threadIdx.x;

    //int const by = blockIdx.y;
    int const ty = threadIdx.y;

    //Use of bx intentional (limits of by)
    uint id = bx * blockDim.y + ty;

    if (id >= obs_rays.num_cols) return;

    int const stride = obs_rays.pitch / sizeof(cacc::Vec3f);
    uint num_rows = min(obs_rays.num_rows_ptr[id], obs_rays.max_rows);

    cacc::Vec3f * rel_rays = obs_rays.data_ptr + id;
    cacc::Vec3f rel_ray;
    int key = tx;
    float theta = -1.0f;
    if (key < num_rows) {
        rel_ray = rel_rays[key * stride]; //16 byte non coaleced read...
        //theta = dot(cacc::Vec3f(0.0f, 0.0f, 1.0f), rel_ray);
        theta = (rel_ray[3] >= 0.0f) ? rel_ray[2] : -1.0f;
    }

    //Bitonic Sort
    for (int i = 0; (1 << i) < num_rows; ++i) {
        bool asc = (tx >> (i + 1)) % 2;

        for (int stride = 1 << i; stride > 0; stride >>= 1) {
            bool dir = (tx % (stride << 1)) < stride;

            float otheta = __shfl_xor(theta, stride);
            int okey = __shfl_xor(key, stride);

            if (otheta != theta && (otheta > theta == (asc ^ dir))) {
                theta = otheta;
                key = okey;
            }
        }
    }

    /* Remove invalid entries */
    uint num_valid = __popc(__ballot(theta >= 0.0f));
    if (num_valid < num_rows && tx == 0) {
        obs_rays.num_rows_ptr[id] = num_valid;
    }

    #pragma unroll
    for (int i = 0; i < 4; ++i) {
        rel_ray[i] = __shfl(rel_ray[i], key);
    }

    if (tx < num_valid) {
        rel_rays[tx * stride] = rel_ray; //16 byte non coalesced write...
    }
}

__global__
void
evaluate_observation_rays(
    cacc::VectorArray<cacc::Vec3f, cacc::DEVICE>::Data obs_rays,
    cacc::Array<float, cacc::DEVICE>::Data recons)
{
    int const bx = blockIdx.x;
    int const tx = threadIdx.x;

    uint id = bx * blockDim.x + tx;

    if (id >= obs_rays.num_cols) return;

    int const stride = obs_rays.pitch / sizeof(cacc::Vec3f);
    uint num_rows = min(obs_rays.num_rows_ptr[id], obs_rays.max_rows);

    cacc::Vec3f * rel_rays = obs_rays.data_ptr + id;

    float recon = num_rows >= 1 ? 0.0f : -1.0f;
    for (int i = 1; i < num_rows; ++i) {
        cacc::Vec3f rel_ray = rel_rays[i * stride];
        recon += heuristic(rel_rays, stride, i, rel_ray);
    }

    recons.data_ptr[id] = recon;
}

__global__
void populate_spherical_histogram(cacc::Vec3f view_pos, float max_distance,
    cacc::BVHTree<cacc::DEVICE>::Accessor const bvh_tree,
    cacc::PointCloud<cacc::DEVICE>::Data cloud,
    cacc::KDTree<3, cacc::DEVICE>::Accessor const kd_tree,
    cacc::Array<float, cacc::DEVICE>::Data sphere_hist)
{
    int const bx = blockIdx.x;
    int const tx = threadIdx.x;

    uint id = bx * blockDim.x + tx;

    if (id >= cloud.num_vertices) return;

    cacc::Vec3f v = cloud.vertices_ptr[id];
    cacc::Vec3f n = cloud.normals_ptr[id];
    cacc::Vec3f v2c = view_pos - v;
    float l = norm(v2c);

    float scale = 1.0f - (l / max_distance);
    if (scale <= 0.0f) return;

    cacc::Vec3f v2cn = v2c / l;
    float ctheta = dot(v2cn, n);
    // 0.087f ~ cos(85.0f / 180.0f * pi)
    if (ctheta < 0.087f) return;

    if (!visible(v, v2cn, l, bvh_tree)) return;

    float capture_difficulty = max(cloud.qualities_ptr[id], 0.0f);

    // 1.484f ~ 85.0f / 180.0f * pi
    float min_theta = min(cloud.values_ptr[id], 1.484f);

    float scaling = (pi / 2.0f) / ((pi / 2.0f) - min_theta);

    float theta = acosf(__saturatef(ctheta));
    float rel_theta = max(theta - min_theta, 0.0f) * scaling;
    float score = capture_difficulty * cosf(rel_theta) * scale;

    uint idx;
    cacc::nnsearch::find_nn<3u>(kd_tree, -v2cn, &idx, nullptr);
    atomicAdd(sphere_hist.data_ptr + idx, score);
}

__global__
void populate_spherical_histogram(cacc::Vec3f view_pos,
    float max_distance, float target_recon,
    cacc::BVHTree<cacc::DEVICE>::Accessor const bvh_tree,
    cacc::PointCloud<cacc::DEVICE>::Data cloud,
    cacc::KDTree<3, cacc::DEVICE>::Accessor const kd_tree,
    cacc::VectorArray<cacc::Vec3f, cacc::DEVICE>::Data obs_rays,
    cacc::Array<float, cacc::DEVICE>::Data recons,
    cacc::Array<float, cacc::DEVICE>::Data sphere_hist)
{
    int const bx = blockIdx.x;
    int const tx = threadIdx.x;

    uint id = bx * blockDim.x + tx;

    if (id >= cloud.num_vertices) return;

    cacc::Vec3f v = cloud.vertices_ptr[id];
    cacc::Vec3f n = cloud.normals_ptr[id];
    cacc::Vec3f v2c = view_pos - v;
    float l = norm(v2c);

    float scale = 1.0f - (l / max_distance);
    if (scale <= 0.0f) return;

    cacc::Vec3f v2cn = v2c / l;
    float ctheta = dot(v2cn, n);
    // 0.087f ~ cos(85.0f / 180.0f * pi)
    if (ctheta < 0.087f) return;

    if (!visible(v, v2cn, l, bvh_tree)) return;

    int const stride = obs_rays.pitch / sizeof(cacc::Vec3f);
    uint num_rows = obs_rays.num_rows_ptr[id];

    if (num_rows >= obs_rays.max_rows) return;

    float recon = recons.data_ptr[id];
    float new_recon = 0.0f;

    if (num_rows >= 1) {
        cacc::Vec3f rel_ray = relative_direction(v2cn, n);
        rel_ray[3] = scale;
        cacc::Vec3f * rel_rays = obs_rays.data_ptr + id;
        new_recon = recon + heuristic(rel_rays, stride, num_rows, rel_ray);
    }

    float delta = func(new_recon, target_recon) - func(recon, target_recon);

    uint idx;
    cacc::nnsearch::find_nn<3u>(kd_tree, -v2cn, &idx, nullptr);
    atomicAdd(sphere_hist.data_ptr + idx, delta);
}

__global__
void
evaluate_spherical_histogram(cacc::Mat3f calib, int width, int height,
    cacc::KDTree<3, cacc::DEVICE>::Accessor const kd_tree,
    cacc::Array<float, cacc::DEVICE>::Data const sphere_hist,
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
    //float theta = (y / (float) hist.height) * pi;
    float theta = (0.5f + (y / (float) hist.height) / 2.0f) * pi;
    float stheta = sinf(theta);
    cacc::Vec3f view_dir(stheta * cosf(phi), stheta * sinf(phi), cosf(theta));
    view_dir.normalize();

    /* Determine stable rotation matrix
     * (local coordinate system with z == viewing direction). */
    cacc::Vec3f rz = -view_dir;

    cacc::Vec3f up = cacc::Vec3f(0.0f, 0.0f, 1.0f);
    bool stable = abs(dot(up, rz)) < 0.99f;
    up = stable ? up : cacc::Vec3f(cosf(phi), sinf(phi), 0.0f);

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

        sum += sphere_hist.data_ptr[i];
    }

    int const stride = hist.pitch / sizeof(float);
    hist.data_ptr[y * stride + x] = sum;
}

__global__ void
estimate_capture_difficulty(float max_distance,
    cacc::BVHTree<cacc::DEVICE>::Accessor const bvh_tree, uint mesh_size,
    cacc::KDTree<3, cacc::DEVICE>::Accessor const kd_tree,
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
    float max_ctheta = 0.0f;

    /* Sample hemisphere (KDTree contains vertices of a sphere). */
    for (uint i = 0; i < kd_tree.num_verts; ++i) {
        cacc::Vec3f dir = kd_tree.verts_ptr[i].normalize();
        float ctheta = dot(dir, n);
        if (ctheta < 0.087f) continue;

        cacc::Ray ray;
        ray.origin = v;
        ray.dir = dir;
        ray.set_tmin(l * 0.001f);
        ray.set_tmax(l);

        uint face_id;
        /* Sample observable? */
        if (!cacc::tracing::trace(bvh_tree, ray, &face_id) || face_id >= mesh_size) {
            if (ctheta > max_ctheta) max_ctheta = ctheta;
        } else {
            sum += ctheta;
        }
    }

    /* Num samples on hemisphere times expectation of sample (derivation in the thesis) */
    float max = (kd_tree.num_verts / 2.0f) * 0.5f;

    float min_theta = acosf(__saturatef(max_ctheta));
    cloud.values_ptr[id] = min_theta;
    cloud.qualities_ptr[id] = sum / max;
}

__global__ void
calculate_func_recons(
    cacc::Array<float, cacc::DEVICE>::Data recons,
    float target_recon,
    cacc::Array<float, cacc::DEVICE>::Data wrecons)
{
    int const bx = blockIdx.x;
    int const tx = threadIdx.x;

    uint id = bx * blockDim.x + tx;
    if (id >= recons.num_values) return;

    float recon = recons.data_ptr[id];
    wrecons.data_ptr[id] = func(recon, target_recon);
}
