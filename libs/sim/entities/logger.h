#ifndef ENTITY_LOGGER_HEADER
#define ENTITY_LOGGER_HEADER

#include "sim/entity.h"

class Logger : public UpdateComponent {
private:
    Pose::ConstPtr pose;
    Motion::ConstPtr motion;
    Trajectory::Ptr trajectory;
    float accum;

public:
    Logger(Pose::ConstPtr pose, Motion::Ptr motion, Trajectory::Ptr trajectory)
        : pose(pose), motion(motion), trajectory(trajectory), accum(0.0f) {};

    void update(double delta_time)
    {
        accum += delta_time;
        if (accum < 1.0f) return;
        accum -= 1.0f;

        trajectory->xs.push_back(pose->x);
        trajectory->qs.push_back(pose->q);
#if 0
        //math::Matrix3f rot;
        //copter.q.to_rotation_matrix(rot.begin());
        //math::Vec3f rates = rot.transposed() * copter.omega;
        std::cout << std::fixed << std::setprecision(4)
            << " Altitude: " << pose->x[2]
            << " Airspeed: " << motion->v.norm()
            //<< " Pitch: " << std::acos(rot.col(0).dot(math::Vec3f(0.0f, 0.0f, 1.0f))) - pi / 2.0f << "(" << rates[1] << ")"
            //<< " Roll: " << std::acos(rot.col(1).dot(math::Vec3f(0.0f, 0.0f, 1.0f))) - pi / 2.0f << "(" << rates[0] << ")"
            //<< " Yaw: " << std::acos(rot.col(0).dot(math::Vec3f(1.0f, 0.0f, 0.0f))) - pi / 2.0f << "(" << rates[2] << ")"
            << std::endl;
#endif
    }
};

#endif /* ENTITY_LOGGER_HEADER */
