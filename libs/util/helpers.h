#include "math/vector.h"

#include "mve/camera.h"

#include "sfm/camera_pose.h"

void fill_camera_pose(mve::CameraInfo const & cam, int width, int height,
    sfm::CameraPose * pose)
{
    math::Matrix3f K(0.0f), R;
    math::Vec3f t;
    cam.fill_camera_translation(t.begin());
    cam.fill_world_to_cam_rot(R.begin());

    float dim_aspect = static_cast<float>(width) / height;
    float img_aspect = dim_aspect * cam.paspect;

    if (img_aspect < 1.0f) {
        K[0] = cam.flen / cam.paspect;
        K[4] = cam.flen;
    } else {
        K[0] = cam.flen;
        K[4] = cam.flen * cam.paspect;
    }

    K[2] = cam.ppoint[0] - 0.5f;
    K[5] = cam.ppoint[1] - 0.5f;
    K[8] = 1.0;

    pose->K = K;
    pose->R = R;
    pose->t = t;
}
