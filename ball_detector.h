#ifndef BALL_DETECTOR_H
#define BALL_DETECTOR_H

#include <vector>
#include <cstdint>

#include <base_detector.h>
#include <object_hypothesis.h>

namespace htwk {

class BallDetector : protected BaseDetector {
public:
    BallDetector(const int width, const int height, const int8_t *lutCb, const int8_t *lutCr) __attribute__((nonnull));

    virtual ~BallDetector();
    virtual void proceed(const uint8_t *img, std::vector<ObjectHypothesis>& hypoList) = 0;
    virtual bool isBallFound() const = 0;
    virtual const ObjectHypothesis& getBall() const = 0;
};

}  // namespace htwk

#endif  // BALL_DETECTOR_H
