#ifndef OBJECT_DETECTOR_H
#define OBJECT_DETECTOR_H

#include <cstdint>
#include <optional>
#include <vector>

#include "ball_feature_extractor.h"
#include "base_detector.h"
#include "htwk_vision_config.h"
#include "object_hypothesis.h"
#include "tfliteexecuter.h"

namespace htwk {

/**
 * @brief The ObjectDetector class detects multiple object via a neural net
 */
class UpperCamPenaltySpotClassifier : public BaseDetector {
public:
    UpperCamPenaltySpotClassifier(int8_t* lutCb, int8_t* lutCr, BallFeatureExtractor* _featureExtractor,
                   HtwkVisionConfig& config) __attribute__((nonnull));
    ~UpperCamPenaltySpotClassifier() = default;

    void proceed(const uint8_t* img, const std::vector<ObjectHypothesis>& hypoList);

    std::optional<ObjectHypothesis> getPenaltySpot() const {
        return penaltySpotHypotheses;
    }

    // This is used by the machine learning tools. This must not be used in the firmware!
    const std::vector<ObjectHypothesis>& getRatedPenaltySpotHypotheses() const {
        return ratedPenaltySpotHypotheses;
    }

private:
    std::optional<ObjectHypothesis> penaltySpotHypotheses;
    std::vector<ObjectHypothesis> ratedPenaltySpotHypotheses;

    BallFeatureExtractor* featureExtractor;
    TFLiteExecuter tflitePenaltyspot;

    int patchSize;
};

}  // namespace htwk

#endif  // OBJECT_DETECTOR_H
