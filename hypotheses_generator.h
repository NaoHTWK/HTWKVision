#ifndef HYPOTHESES_GENERATOR_H
#define HYPOTHESES_GENERATOR_H

#include <cstdint>

#include <vector>

#include "field_color_detector.h"
#include "integral_image.h"
#include "object_hypothesis.h"

namespace htwk {

class HypothesesGenerator
{
public:
    HypothesesGenerator();
    virtual ~HypothesesGenerator();

    virtual void proceed(uint8_t *img, const int * const fieldborder, float camPitch, float camRoll, IntegralImage *integralImg) = 0;
    virtual std::vector<ObjectHypothesis> getHypotheses() const = 0;
    virtual int* getRatingImg() = 0;
};

} // htwk

#endif // HYPOTHESES_GENERATOR_H
