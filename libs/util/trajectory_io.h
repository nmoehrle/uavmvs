#ifndef UTIL_TRAJECTORY_IO_HEADER
#define UTIL_TRAJECTORY_IO_HEADER

#include <fstream>

#include "mve/camera.h"

#include "sim/entity.h"

void save_trajectory(Trajectory::ConstPtr trajectory, std::string const & path) {
    std::ofstream out(path.c_str());
    if (!out.good()) throw std::runtime_error("Could not open file");
    std::size_t length = trajectory->xs.size() & trajectory->qs.size();
    out << length << std::endl;
    for (std::size_t i = 0; i < length; ++i) {
        out << trajectory->xs[i] << std::endl;

        math::Matrix3f rot;
        trajectory->qs[i].to_rotation_matrix(rot.begin());
        out << rot;
    }
    out.close();
}

void save_trajectory(std::vector<mve::CameraInfo> const & trajectory,
    std::string const & path)
{
    std::ofstream out(path.c_str());
    if (!out.good()) throw std::runtime_error("Could not open trajectory file for writing");
    std::size_t length = trajectory.size();
    out << length << std::endl;

    for (std::size_t i = 0; i < length; ++i) {
        mve::CameraInfo const & cam = trajectory[i];
        math::Vec3f trans(cam.trans);
        math::Matrix3f rot(cam.rot);
        math::Vec3f pos = -rot.transposed() * trans;

        out << pos << std::endl;
        out << rot;
    }

    out.close();
}

void load_trajectory(std::string const & path,
    std::vector<mve::CameraInfo> * trajectory)
{
    std::ifstream in(path.c_str());
    if (!in.good()) throw std::runtime_error("Could not open trajectory file");
    std::size_t length;
    in >> length;

    trajectory->resize(length);

    for (std::size_t i = 0; i < length; ++i) {
        math::Vec3f pos;
        for (int j = 0; j < 3; ++j) {
            in >> pos[j];
        }
        math::Matrix3f rot;
        for (int j = 0; j < 9; ++j) {
            in >> rot[j];
        }
        math::Vec3f trans = -rot * pos;

        mve::CameraInfo & cam = trajectory->at(i);
        cam.flen = 0.86f; //TODO save and read from file
        std::copy(trans.begin(), trans.end(), cam.trans);
        std::copy(rot.begin(), rot.end(), cam.rot);
    }

    if (in.fail()) {
        in.close();
        throw std::runtime_error("Invalid trajectory file");
    }

    in.close();
}

#endif /* UTIL_TRAJECTORY_IO_HEADER */
