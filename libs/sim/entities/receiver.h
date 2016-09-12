#ifndef ENTITY_RECEIVER_HEADER
#define ENTITY_RECEIVER_HEADER

#include "sim/entity.h"

class Receiver : public UpdateComponent {
private:
    Control::Ptr control;

public:
    Receiver(Control::Ptr control) : control(control) {}

    void update(double) {
        if (glfwJoystickPresent(GLFW_JOYSTICK_1)) {
            int count;
            const float * axes = glfwGetJoystickAxes(GLFW_JOYSTICK_1, &count);
            float turn_rate = 1.0f;
            float yaw_rate = axes[3] * turn_rate;
            float pitch_rate = -axes[1] * turn_rate;
            float roll_rate = -axes[0] * turn_rate;
            control->throttle = (1.0f + axes[2]) / 2.0f;
            control->omega = math::Vec3f(roll_rate, pitch_rate, yaw_rate);
        } else {
            control->throttle = 0.0f;
            control->omega = math::Vec3f(0.0f);
        }
    }
};

#endif /* ENTITY_RECEIVER_HEADER */
