#ifndef GEOM_VOLUME_HEADER
#define GEOM_VOLUME_HEADER

#include "math/vector.h"

#include "mve/image.h"

template <typename IdxType>
class Volume {
public:
    typedef std::shared_ptr<Volume> Ptr;
    typedef std::shared_ptr<const Volume> ConstPtr;

private:
    math::Vector<IdxType, 3> dim;
    math::Vec3f resolution;
    math::Vec3f min;
    math::Vec3f max;
    std::vector<mve::FloatImage::Ptr> values;

public:
    Volume(IdxType width, IdxType height, IdxType depth,
        math::Vec3f min, math::Vec3f max)
        : dim(width, height, depth), min(min), max(max) {
        //static_assert(std::is_integral<IdxType>::value, "IdxType must be an integer type.");
        //static_assert(std::is_unsigned<IdxType>::value, "IdxType must be an unsigned type.");
        resolution = (max - min).cw_div(math::Vec3f(width, height, depth));
        values.resize(width * height * depth);
    }

    static Ptr create(IdxType width, IdxType height, IdxType depth,
        math::Vec3f min, math::Vec3f max) {
        return std::make_shared<Volume>(width, height, depth, min, max);
    }

    math::Vec3f minimum(void) const { return min; }
    math::Vec3f maximum(void) const { return max; }
    IdxType width(void) const { return dim[0]; }
    IdxType height(void) const { return dim[1]; }
    IdxType depth(void) const { return dim[2]; }
    math::Vector<IdxType, 3> dimension() const { return dim; }

    math::Vec3f position(math::Vector<IdxType, 3> pos) {
        return min + resolution.cw_mult(math::Vec3f(pos));
    }

    math::Vec3f position(IdxType x, IdxType y, IdxType z) {
        return min + resolution.cw_mult(math::Vec3f(x, y, z));
    }

    IdxType num_positions(void) const {
        return width() * height() * depth();
    }

    IdxType index(math::Vector<IdxType, 3> pos) const {
        return (pos[2] * dim[1] + pos[1]) * dim[0] + pos[0];
    }

    IdxType index(IdxType x, IdxType y, IdxType z) const {
        return (z * height() + y) * width() + x;
    }

    mve::FloatImage::Ptr & at(IdxType idx) {
        return values[idx];
    }

    mve::FloatImage::ConstPtr at(IdxType idx) const {
        return values[idx];
    }

    mve::FloatImage::Ptr & at(math::Vector<IdxType, 3> pos) {
        return values[index(pos)];
    }

    mve::FloatImage::Ptr & at(IdxType x, IdxType y, IdxType z) {
        return values[index(x, y, z)];
    }
};

#endif /* GEOM_VOLUME_HEADER */
