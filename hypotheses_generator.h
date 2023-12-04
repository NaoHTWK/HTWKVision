#ifndef HYPOTHESES_GENERATOR_H
#define HYPOTHESES_GENERATOR_H

#include <cstdint>

#include <vector>

#include "field_color_detector.h"
#include "integral_image.h"
#include "object_hypothesis.h"
#include <localization_utils.h>

namespace htwk {

class HypothesesGenerator
{
public:
    HypothesesGenerator() = default;
    HypothesesGenerator(const HypothesesGenerator&) = delete;
    HypothesesGenerator(HypothesesGenerator&&) = delete;
    HypothesesGenerator& operator=(const HypothesesGenerator&) = delete;
    HypothesesGenerator& operator=(HypothesesGenerator&&) = delete;
    virtual ~HypothesesGenerator() = default;

    virtual void proceed(uint8_t *img, const std::vector<int>& fieldborder, CamPose& cam_pose, IntegralImage *integralImg) = 0;
    virtual std::vector<ObjectHypothesis> getHypotheses() const = 0;
    virtual int* getRatingImg() = 0;
    virtual uint8_t* getDebugImg() = 0;
    virtual void setDebugActive(bool active) = 0;

};

} // htwk

#endif // HYPOTHESES_GENERATOR_H
