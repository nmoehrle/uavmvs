#include "cacc/tracing.h"

#include "kernel.h"

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
kernel(cacc::Mat4f w2c, cacc::Mat3f calib, cacc::Vec3f view_pos,
    int width, int height,
    cacc::BVHTree<cacc::DEVICE>::Data bvh_tree,
    cacc::PointCloud<cacc::DEVICE>::Data cloud,
    cacc::VectorArray<cacc::DEVICE, cacc::Vec2f>::Data dir_hist)
{
    int const bx = blockIdx.x;
    int const tx = threadIdx.x;

    uint id = bx * blockDim.x + tx;

    if (id >= cloud.num_vertices) return;
    cacc::Vec3f v = cloud.vertices_ptr[id];
    cacc::Vec3f v2c = view_pos - v;
    float n = norm(v2c);
    if (n > 45.0f) return; //TODO make configurable
    cacc::Vec2f p = project(v, w2c, calib);

    if (p[0] < 0.0f || width <= p[0] || p[1] < 0.0f || height <= p[1]) return;

    cacc::Ray ray;
    ray.origin = v + v2c * 0.01f;
    ray.dir = v2c / n;
    ray.set_tmin(0.0f);
    ray.set_tmax(inf);

    uint hit_face_id;
    cacc::tracing::trace(bvh_tree, ray, &hit_face_id);
    if (hit_face_id != cacc::tracing::NAI) return;

    uint row = dir_hist.num_rows_ptr[id] + 1;

    if (row > dir_hist.max_rows) return;

    dir_hist.num_rows_ptr[id] = row;
    cacc::Vec2f dir(atan2(ray.dir[1], ray.dir[0]), acos(ray.dir[2]));
    dir_hist.data_ptr[row * dir_hist.pitch + id] = dir;
}
