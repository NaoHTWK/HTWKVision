#pragma once

#include <point_2d.h>

namespace htwk {

struct LineCross {
    point_2d p;
    point_2d v;

    LineCross(const point_2d& p, const point_2d& v) : p(p), v(v) {}
};

}  // namespace htwk
