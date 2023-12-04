#ifndef BALL_DETECTOR_H
#define BALL_DETECTOR_H

#include <cstdint>
#include <optional>
#include <vector>

#include <base_detector.h>
#include <object_hypothesis.h>

namespace htwk {

class BallDetector : protected BaseDetector {
public:
    using BaseDetector::BaseDetector;
    BallDetector(const BallDetector&) = delete;
    BallDetector(const BallDetector&&) = delete;
    BallDetector& operator=(const BallDetector&) = delete;
    BallDetector& operator=(const BallDetector&&) = delete;
    virtual ~BallDetector() = default;

    // This can be empty when there was no ball in the image.
    virtual const std::optional<ObjectHypothesis>& getBall() const = 0;

    // This is used by the machine learning tools. This must not be used in the firmware!
    virtual const std::vector<ObjectHypothesis>& getRatedBallHypotheses() const = 0;

    virtual const std::vector<ObjectHypothesis>& getAllHypothesesWithProb() const = 0;
};

}  // namespace htwk

#endif  // BALL_DETECTOR_H
