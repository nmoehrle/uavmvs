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

struct DataComponent {
    typedef std::shared_ptr<const DataComponent> ConstPtr;
};

class UpdateComponent {
    public:
        typedef std::shared_ptr<UpdateComponent> Ptr;
        virtual void update(double delta_time) = 0;
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

struct PhysicalProperties : public DataComponent {
    typedef std::shared_ptr<PhysicalProperties> Ptr;
    typedef std::shared_ptr<const PhysicalProperties> ConstPtr;
    float mass;
    math::Matrix4f I;
    float drag_coeff;
};

class PhysicsSimulator : public UpdateComponent {
    private:
        Pose::Ptr pose;
        Motion::Ptr motion;
        Propulsion::ConstPtr propulsion;
        PhysicalProperties::ConstPtr physical_properties;

    public:
        PhysicsSimulator(Pose::Ptr pose, Motion::Ptr motion,
            Propulsion::ConstPtr propulsion, PhysicalProperties::ConstPtr physical_properties)
            : pose(pose), motion(motion), propulsion(propulsion),
            physical_properties(physical_properties) {};

        void update(double delta_time) {
            //TODO
        }
};


class FlightController : public UpdateComponent {
    private:
        Pose::ConstPtr pose;
        Motion::ConstPtr motion;
        Propulsion::Ptr propulsion;

        math::Vec3f * rates; //TODO
    public:
        FlightController(Pose::ConstPtr pose, Motion::ConstPtr motion, Propulsion::Ptr propulsion)
            : pose(pose), motion(motion), propulsion(propulsion) {};

        void update(double delta_time) {
            //TODO
        }
};

class Logger : public UpdateComponent {
    private:
        Trajectory::Ptr trajectory;
        Pose::ConstPtr pose;

    public:
        Logger(Pose::ConstPtr pose, Trajectory::Ptr trajectory)
            : pose(pose), trajectory(trajectory) {};

        void update(double delta_time)
        {
            //TODO only every x milliseconds;
            trajectory->xs.push_back(pose->x);
        }
};

#endif // ENTITY_HEADER
