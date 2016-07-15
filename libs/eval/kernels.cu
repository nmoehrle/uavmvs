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

    uint hit_face_id;
    cacc::tracing::trace(bvh_tree, ray, &hit_face_id);
    return hit_face_id != cacc::tracing::NAI;
}

__forceinline__ __device__
cacc::Vec2f
relative_angle(cacc::Vec3f const & v2cn, cacc::Vec3f const & n, float ctheta) {
    cacc::Vec3f b0 = orthogonal(n).normalize();
    cacc::Vec3f b1 = cross(n, b0).normalize();
    cacc::Vec3f x = v2cn - ctheta * n;
    return cacc::Vec2f(dot(x, b0), dot(x, b1));
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
    cacc::Vec3f n = cloud.normals_ptr[id];
    cacc::Vec3f v2c = view_pos - v;
    float l = norm(v2c);
    cacc::Vec3f v2cn = v2c / l;

    float ctheta = dot(v2cn, n);
    // 0.087f ~ cos(85.0f / 180.0f * pi)
    //if (abs(ctheta) < 0.087f) return;

    //if (l > 80.0f) return; //TODO make configurable
    cacc::Vec2f p = project(v, w2c, calib);

    if (p[0] < 0.0f || width <= p[0] || p[1] < 0.0f || height <= p[1]) return;

    if (!visible(v, v2cn, l, bvh_tree)) return;

    uint row = dir_hist.num_rows_ptr[id];

    if (row >= dir_hist.max_rows) return;

    cacc::Vec2f dir = relative_angle(v2cn, n, ctheta);

    dir_hist.num_rows_ptr[id] = row;
    //cacc::Vec2f dir(atan2(ray.dir[1], ray.dir[0]), acos(ray.dir[2]));
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
    return cov;
    //return cov / (float)n;
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
    cacc::Vec2f mu = mean(values, stride, n);
    cacc::Mat2f cov = covariance(values, stride, n, mu);
    return eigen_values(cov);
}

__global__
void
evaluate_histogram(cacc::VectorArray<cacc::DEVICE, cacc::Vec2f>::Data dir_hist)
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
    cacc::VectorArray<cacc::DEVICE, cacc::Vec2f>::Data dir_hist,
    cacc::Image<float, cacc::DEVICE>::Data hist) {

    int const bx = blockIdx.x;
    int const tx = threadIdx.x;

    uint id = bx * blockDim.x + tx;

    if (id >= cloud.num_vertices) return;
    cacc::Vec3f v = cloud.vertices_ptr[id];
    cacc::Vec3f n = cloud.normals_ptr[id];
    cacc::Vec3f v2c = view_pos - v;
    float l = norm(v2c);
    cacc::Vec3f v2cn = v2c / l;

    //if (l > 80.0f) return; //TODO make configurable
    if (!visible(v, v2cn, l, bvh_tree)) return;

    float ctheta = dot(v2cn, n);
    cacc::Vec2f dir = relative_angle(v2cn, n, ctheta);

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

    float & r = l;
    float theta = acos(v2cn[2] / r);
    float phi = atan2(v2cn[1], v2cn[0]);

    //57.295779 ~ 180.0 / pi
    int x = theta * 57.295780;
    int y = phi * 57.295780;

    float * bin = hist.data_ptr + y * hist.pitch / sizeof(float) + x;
    atomicAdd(bin, contrib);
}
