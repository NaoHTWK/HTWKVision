#ifndef __GOALPOST_H__
#define __GOALPOST_H__

#include <color.h>
#include <point_2d.h>
#include <vector>

namespace htwk {

struct GoalPost {
    GoalPost(int x, int y) : basePoint(x, y) {}
    GoalPost(point_2d bPoint) : basePoint(bPoint) {}
    point_2d basePoint;
    point_2d upperPoint;
    int width;
    htwk::color color;
    float probability;
};

}  // namespace htwk

#endif // __GOALPOST_H__
