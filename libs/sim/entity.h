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
    float max_thrust;
};

struct Trajectory : public DataComponent {
    typedef std::shared_ptr<Trajectory> Ptr;
    typedef std::shared_ptr<const Trajectory> ConstPtr;
    //std::vector<double> ts
    std::vector<math::Vec3f> xs;
    std::vector<math::Quatf> qs;
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

#endif /* ENTITY_HEADER */
