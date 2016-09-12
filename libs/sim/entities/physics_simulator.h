#ifndef ENTITY_PHYSICS_SIMULATOR_HEADER
#define ENTITY_PHYSICS_SIMULATOR_HEADER

#include "sim/entity.h"

class PhysicsSimulator : public UpdateComponent {
private:
    Pose::Ptr pose;
    Motion::Ptr motion;
    Physics::ConstPtr physics;
    Propulsion::ConstPtr propulsion;

    float accum;

    void integrate(float h) {
        math::Matrix3f R;
        pose->q.to_rotation_matrix(R.begin());
        math::Vec3f up = R.col(2);

        float gamma = 0.1f; //TODO estimate
        float air_speed = motion->v.norm();

        // Gravity
        math::Vec3f f = math::Vec3f(0.0, 0.0f, -9.81f * physics->mass);

        // Drag
        float csarea = 0.15f; //0.2f + 0.1f;//calculate properly
        if (air_speed > 0.0f) {
            f += -motion->v.normalized() * physics->drag_coeff * csarea * air_speed * air_speed;
        }

        // Propulsion
        math::Vec3f tau = math::Vec3f(0.0f);
        for (std::size_t i = 0; i < 6; ++i) {
            math::Vec3f thrust = up * propulsion->thrusts[i];
            f += thrust;
            tau += (R * -propulsion->rs[i]).cross(thrust);
            // Every second engine spins ccw
            tau += i % 2 ? -gamma * thrust : gamma * thrust;
        }

        //TODO replace Newton
        pose->x += motion->v * h;
        motion->v += (f / physics->mass) * h;
        motion->omega += R * physics->Ii * R.transposed() * tau * h;

        math::Vec3f qs = R.transposed() * motion->omega * h;
        float theta = qs.norm();

        if (std::abs(theta) >= std::numeric_limits<float>::epsilon()) {
            math::Vec3f e(qs[0] / theta, qs[1] / theta, qs[2] / theta);
            math::Quaternion<float> qd(e, theta);
            pose->q = qd * pose->q;
        }
    }

public:
    PhysicsSimulator(Pose::Ptr pose, Motion::Ptr motion,
        Physics::ConstPtr physics, Propulsion::ConstPtr propulsion)
        : pose(pose), motion(motion), physics(physics),
        propulsion(propulsion), accum(0.0f) {};

    void update(double delta_time)
    {
        accum += delta_time;
        while (accum >= time_step) {
            integrate(time_step);
            accum -= time_step;
        }
    }

};

#endif /* ENTITY_PHYSICS_SIMULATOR_HEADER */
