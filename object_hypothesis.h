#ifndef OBJECT_HYPOTHESIS_H
#define OBJECT_HYPOTHESIS_H

#include <integral_image.h>
#include <point_2d.h>

namespace htwk {

enum class ObjectType { NONE = 0, PENALTY_SPOT, GOAL_POST, CENTER_SPOT, CIRCLE_CROSSING };

class ObjectHypothesis {
public:
    int x{}, y{}, r{}, rating{};
    float prob{};
    ObjectType type = ObjectType::NONE;

    ObjectHypothesis() = default;
    ObjectHypothesis(point_2d center, int r, int rating = 0)
        : x((int)center.x), y((int)center.y), r(r), rating(rating) {}
    ObjectHypothesis(int x, int y, int r, int rating) : x(x), y(y), r(r), rating(rating) {}
    ObjectHypothesis(const ObjectHypothesis& o, ObjectType type)
        : x(o.x), y(o.y), r(o.r), rating(o.rating), type(type) {}
    point_2d point() const {
        return {static_cast<float>(x), static_cast<float>(y)};
    }
};
}  // namespace htwk
#endif  // OBJECT_HYPOTHESIS_H
