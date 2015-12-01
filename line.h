#ifndef LINE_H
#define LINE_H

#include "coord.h"

namespace htwk {

struct Line{
    inline Coord getIntersection(const Line &other) const {
        float ax = px1 - px2;
        float ay = py1 - py2;
        float bx = other.px1 - other.px2;
        float by = other.py1 - other.py2;
        float ab = ax * -by + ay * bx;

        if( ab == 0 ) {
            return newCoord( 0, 0 );
        }

        float qb_ = other.px1 * -by + other.py1 * bx;
        float pa_ = px1 * -ay + py1 * ax;
        return newCoord( 1.f / ab * ( qb_ * ax - pa_ * bx ),
                         1.f / ab * ( qb_ * ay - pa_ * by ) );
    }

    float px1, py1, px2, py2;
    int id;
};

inline Line newLine(const Coord& p1, const Coord& p2 ){
    Line l;
    l.px1=p1.x;
    l.py1=p1.y;
    l.px2=p2.x;
    l.py2=p2.y;
    l.id=0;
    return l;
}

inline Line newLine(float dpx1, float dpy1, float dpx2, float dpy2, int iid){
    Line l;
    l.px1=dpx1;
    l.py1=dpy1;
    l.px2=dpx2;
    l.py2=dpy2;
    l.id=iid;
    return l;
}

inline Line newLine(float dpx1, float dpy1, float dpx2, float dpy2){
    Line l;
    l.px1=dpx1;
    l.py1=dpy1;
    l.px2=dpx2;
    l.py2=dpy2;
    l.id=0;
    return l;
}

}

#endif // LINE_H
