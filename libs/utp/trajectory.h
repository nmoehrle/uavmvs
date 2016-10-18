#ifndef UTP_TRAJECTORY_HEADER
#define UTP_TRAJECTORY_HEADER

#include <vector>

#include "math/vector.h"
#include "math/matrix.h"

#include "mve/camera.h"

#include "defines.h"

UTP_NAMESPACE_BEGIN

typedef std::vector<mve::CameraInfo> Trajectory;

math::Matrix3f rotation_from_spherical(float theta, float phi);

std::pair<float, float> spherical_from_rotation(math::Matrix3f const & rot);

float length(Trajectory const & trajectory);

UTP_NAMESPACE_END

#endif /* UTP_TRAJECTORY_HEADER */
