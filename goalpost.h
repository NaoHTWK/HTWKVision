#ifndef __GOALPOST_H__
#define __GOALPOST_H__

#include <vector>

#include "color.h"
#include "point_2d.h"

namespace htwk {

struct GoalPost {
    GoalPost(float x, float y) : basePoint(x, y) {}
    GoalPost(point_2d bPoint) : basePoint(bPoint) {}
    point_2d basePoint;
    point_2d upperPoint;
    int width;
    htwk::color color;
    float probability;
};

}  // namespace htwk

#endif // __GOALPOST_H__
