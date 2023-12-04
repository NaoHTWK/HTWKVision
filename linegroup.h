#pragma once

#include <line.h>
#include <lineedge.h>

namespace htwk {

struct LineGroup{
	LineEdge lines[2];
    int points {0};

    LineGroup() = default;
    Line middle() const { return {(lines[0].p1() + lines[1].p1()) / 2.f, (lines[0].p2() + lines[1].p2()) / 2.f}; }
};

}  // namespace htwk
