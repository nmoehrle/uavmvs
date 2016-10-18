#ifndef UTP_TRAJECTORY_IO_HEADER
#define UTP_TRAJECTORY_IO_HEADER

#include <fstream>

#include "math/vector.h"
#include "math/matrix.h"

#include "trajectory.h"

UTP_NAMESPACE_BEGIN

void save_trajectory(Trajectory const & trajectory,
    std::string const & path);

void load_trajectory(std::string const & path,
    Trajectory * trajectory);

UTP_NAMESPACE_END

#endif /* UTP_TRAJECTORY_IO_HEADER */
