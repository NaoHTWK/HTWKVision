#ifndef BALL_OBJECT_DETECTOR_H
#define BALL_OBJECT_DETECTOR_H

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "ball_detector.h"
#include "ball_feature_extractor.h"
#include "base_detector.h"
#include "field_border_detector.h"
#include "htwk_vision_config.h"
#include "integral_image.h"
#include "object_hypothesis.h"
#include "tfliteexecuter.h"

namespace htwk {

/**
 * @brief The BallObjectDetector class detects the ball as a object via a neural net
 */
class BallPreClassifierUpperCam : public BallDetector {
public:
    BallPreClassifierUpperCam(int8_t* lutCb, int8_t* lutCr, BallFeatureExtractor* featureExtractor, HtwkVisionConfig& config);
    ~BallPreClassifierUpperCam() override = default;

    void proceed(const uint8_t* img, std::shared_ptr<FieldBorderDetector> fieldBorderDetector, std::vector<ObjectHypothesis>& hypoList);

    const std::optional<ObjectHypothesis>& getBall() const override {
        return bestBallHypothesis;
    }

    // This is used by the machine learning tools. This must not be used in the firmware!
    const std::vector<ObjectHypothesis>& getRatedBallHypotheses() const override {
        return ratedBallHypotheses;
    }

    const std::vector<ObjectHypothesis>& getAllHypothesesWithProb() const override {
        return allHypothesesWithProb;
    }

private:
    std::optional<ObjectHypothesis> bestBallHypothesis;

    std::vector<ObjectHypothesis> ratedBallHypotheses;
    std::vector<ObjectHypothesis> allHypothesesWithProb;

    BallFeatureExtractor* featureExtractor;
    TFLiteExecuter tflite;
};

}  // namespace htwk

#endif  // BALL_OBJECT_DETECTOR_H
