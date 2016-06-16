#ifndef ENTITY_HEADER
#define ENTITY_HEADER

#include <memory>
#include <vector>

#include "math/vector.h"
#include "math/matrix.h"
#include "math/quaternion.h"

namespace math {
    typedef Quaternion<float> Quatf;
};

//TODO evalute switching from shared pointer to pointer - ownership entity

struct DataComponent {
    typedef std::shared_ptr<const DataComponent> ConstPtr;
    virtual ~DataComponent() {}
};

class UpdateComponent {
public:
    typedef std::shared_ptr<UpdateComponent> Ptr;
    virtual void update(double delta_time) = 0;
    virtual ~UpdateComponent() {}
};

class RenderComponent {
public:
    typedef std::shared_ptr<RenderComponent> Ptr;
    virtual void render() = 0;
    virtual ~RenderComponent() {}
};

class Entity {
public:
    typedef std::shared_ptr<Entity> Ptr;

private:
    std::vector<DataComponent::ConstPtr> data;
    std::vector<UpdateComponent::Ptr> updaters;
    std::vector<RenderComponent::Ptr> renderers;

public:
    void add_component(DataComponent::ConstPtr component)
    {
        data.push_back(component);
    }

    void add_component(UpdateComponent::Ptr component)
    {
        updaters.push_back(component);
    }

    void add_component(RenderComponent::Ptr component)
    {
        renderers.push_back(component);
    }

    void update(double delta_time) {
        for (UpdateComponent::Ptr const & updater : updaters) {
            updater->update(delta_time);
        }
    }

    void render() {
        for (RenderComponent::Ptr const & renderer : renderers) {
            renderer->render();
        }
    }
};

struct Pose : public DataComponent {
    typedef std::shared_ptr<Pose> Ptr;
    typedef std::shared_ptr<const Pose> ConstPtr;
    math::Vec3f x; //Position
    math::Quatf q; //Orientation
};

struct Motion : public DataComponent {
    typedef std::shared_ptr<Motion> Ptr;
    typedef std::shared_ptr<const Motion> ConstPtr;
    math::Vec3f v; //Velocity
    math::Vec3f omega; //Angular velocity
};

struct Propulsion : public DataComponent {
    typedef std::shared_ptr<Propulsion> Ptr;
    typedef std::shared_ptr<const Propulsion> ConstPtr;
    std::vector<math::Vec3f> rs;
    std::vector<float> thrusts;
};

struct Trajectory : public DataComponent {
    typedef std::shared_ptr<Trajectory> Ptr;
    typedef std::shared_ptr<const Trajectory> ConstPtr;
    //std::vector<double> ts
    std::vector<math::Vec3f> xs;
};

struct Physics : public DataComponent {
    typedef std::shared_ptr<Physics> Ptr;
    typedef std::shared_ptr<const Physics> ConstPtr;
    float mass;
    math::Matrix3f Ii;
    float drag_coeff;
};

struct Control : public DataComponent {
    typedef std::shared_ptr<Control> Ptr;
    typedef std::shared_ptr<const Control> ConstPtr;
    math::Vec3f omega;
    float throttle;
};

constexpr float time_step = 1.0f / 60.0f;

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
        float p = 0.5f, i = 0.03f, d = 0.04f;

        math::Vec3f out = p * error + i * integral + d * derivative;

        float rate_limit = 0.5f;

        out[0] = std::max(-rate_limit, std::min(rate_limit, out[0]));
        out[1] = std::max(-rate_limit, std::min(rate_limit, out[1]));
        out[2] = std::max(-rate_limit, std::min(rate_limit, out[2]));

        float throttle = 2 * control->throttle * 9.81f;

        propulsion->thrusts[0] = throttle * (1.0f - out[2]) * (1.0f + 0.25f * out[0]) * (1.0f - out[1]);
        propulsion->thrusts[1] = throttle * (1.0f + out[2]) * (1.0f + 0.50f * out[0]);
        propulsion->thrusts[2] = throttle * (1.0f - out[2]) * (1.0f + 0.25f * out[0]) * (1.0f + out[1]);
        propulsion->thrusts[3] = throttle * (1.0f + out[2]) * (1.0f - 0.25f * out[0]) * (1.0f + out[1]);
        propulsion->thrusts[4] = throttle * (1.0f - out[2]) * (1.0f - 0.50f * out[0]);
        propulsion->thrusts[5] = throttle * (1.0f + out[2]) * (1.0f - 0.25f * out[0]) * (1.0f - out[1]);
    }
};

class Logger : public UpdateComponent {
private:
    Pose::ConstPtr pose;
    Motion::ConstPtr motion;
    Trajectory::Ptr trajectory;

public:
    Logger(Pose::ConstPtr pose, Motion::Ptr motion, Trajectory::Ptr trajectory)
        : pose(pose), motion(motion), trajectory(trajectory) {};

    void update(double)
    {
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
        //TODO only every x milliseconds;
        trajectory->xs.push_back(pose->x);
    }
};

#endif // ENTITY_HEADER
