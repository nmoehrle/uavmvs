/*
 * Copyright (C) 2015, Nils Moehrle
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#ifndef ENTITY_FLIGHT_CONTROLLER_HEADER
#define ENTITY_FLIGHT_CONTROLLER_HEADER

#include "sim/entity.h"

class FlightController : public UpdateComponent {
private:
    Pose::ConstPtr pose;
    Motion::ConstPtr motion;
    Control::ConstPtr control;
    Propulsion::Ptr propulsion;

    math::Vec3f last_error;
    math::Vec3f last_integral;
public:
    FlightController(Pose::ConstPtr pose, Motion::ConstPtr motion, Control::Ptr control,
        Propulsion::Ptr propulsion)
        : pose(pose), motion(motion), control(control), propulsion(propulsion),
        last_error(0.0f), last_integral(0.0f) {};

    void update(double delta_time) {
        math::Matrix3f R;
        pose->q.to_rotation_matrix(R.begin());
        math::Vec3f error = R.transposed() * motion->omega - control->omega;

        math::Vec3f integral = last_integral + error * delta_time;
        last_integral = integral;

        math::Vec3f derivative = (error - last_error) / delta_time;

        //TODO extract PID
        float p = 0.5f, i = 0.1f, d = 0.04f;

        math::Vec3f out = p * error + i * integral + d * derivative;

        float rate_limit = 2.0f;

        out[0] = std::max(-rate_limit, std::min(rate_limit, out[0]));
        out[1] = std::max(-rate_limit, std::min(rate_limit, out[1]));
        out[2] = std::max(-rate_limit, std::min(rate_limit, out[2]));

        float thrust = control->throttle * propulsion->max_thrust;

        propulsion->thrusts[0] = thrust * (1.0f - out[2]) * (1.0f + 0.25f * out[0]) * (1.0f - out[1]);
        propulsion->thrusts[1] = thrust * (1.0f + out[2]) * (1.0f + 0.50f * out[0]);
        propulsion->thrusts[2] = thrust * (1.0f - out[2]) * (1.0f + 0.25f * out[0]) * (1.0f + out[1]);
        propulsion->thrusts[3] = thrust * (1.0f + out[2]) * (1.0f - 0.25f * out[0]) * (1.0f + out[1]);
        propulsion->thrusts[4] = thrust * (1.0f - out[2]) * (1.0f - 0.50f * out[0]);
        propulsion->thrusts[5] = thrust * (1.0f + out[2]) * (1.0f - 0.25f * out[0]) * (1.0f - out[1]);
    }
};

#endif /* ENTITY_FLIGHT_CONTROLLER_HEADER */
