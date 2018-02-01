#ifndef OBJECT_HYPOTHESIS_H
#define OBJECT_HYPOTHESIS_H

#include <integral_image.h>

namespace htwk{
struct block{
    int x,y,diff;
};

class ObjectHypothesis
{
public:
    int x,y,r,rating;
    float prob;

    ObjectHypothesis()
        : x(0), y(0), r(0), rating(0), prob(0) {}
    ObjectHypothesis(int _x,int _y,int _r,int _rating)
        : x(_x), y(_y), r(_r), rating(_rating), prob(0) {}
};
}//namespace htwk
#endif // OBJECT_HYPOTHESIS_H
