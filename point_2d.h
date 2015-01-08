#ifndef POINT_2D_H
#define POINT_2D_H

#include <cfloat>
#include <cmath>

namespace htwk {

struct point_2d {
    float x;
    float y;

    bool operator==(const point_2d& o) const {
        return fabs(x - o.x) < FLT_EPSILON && fabs(y - o.y) < FLT_EPSILON;
    }

    inline float magnitude() {
        return sqrt(x*x+y*y);
    }
};

inline point_2d newPoint2D(float x = 0, float y = 0)
{
    struct point_2d temp;
    temp.x = x;
    temp.y = y;
    return temp;
}

}  // namespace htwk

#endif // POINT_2D_H
