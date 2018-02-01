#ifndef OBJECT_DETECTOR_H
#define OBJECT_DETECTOR_H

#include <cstdint>
#include <vector>

#include "ball_detector.h"
#include "ball_feature_extractor.h"
#include "base_detector.h"
#include "htwk_vision_config.h"
#include "integral_image.h"
#include "object_hypothesis.h"

#include <caffe/caffe.hpp>

namespace htwk {

/**
 * @brief The ObjectDetector class detects multiple object via a neural net
 */
class ObjectDetector : public BallDetector {

private:
    const float& MIN_BALL_PROB;
    const float& MIN_PENALTYSPOT_PROB;
    const float& MIN_FEET_PROB;

    const int FEATURE_SIZE;

    bool foundBall;
    bool foundPenaltySpot;
    int foundFeet;

    ObjectHypothesis bestBallHypothesis;
    ObjectHypothesis bestPenaltySpotHypothesis;

    std::vector<ObjectHypothesis> ratedBallHypotheses;
    std::vector<ObjectHypothesis> ratedPenaltySpotHypotheses;
    std::vector<ObjectHypothesis> ratedFeetHypotheses;

    BallFeatureExtractor* featureExtractor;

    std::shared_ptr<caffe::Net<float>> classifier;
    caffe::Blob<float>* inputLayer;
    caffe::Blob<float>* outputLayer;

public:

    ObjectDetector(int width, int height,
                 int8_t *lutCb, int8_t *lutCr,
                 BallFeatureExtractor* _featureExtractor, const HtwkVisionConfig& config) __attribute__((nonnull));
    void proceed(const uint8_t *img, std::vector<ObjectHypothesis> &hypoList) override;

    bool isBallFound() const override { return foundBall; }
    bool isPenaltySpotFound() const { return foundPenaltySpot; }
    int feetsFound() const { return foundFeet; }

    const ObjectHypothesis& getBall() const override { return bestBallHypothesis; }
    const ObjectHypothesis& getPenaltySpot() const { return bestPenaltySpotHypothesis; }

    // This is used by the machine learning tools. This must not be used in the firmware!
    const std::vector<ObjectHypothesis>& getRatedBallHypotheses() const { return ratedBallHypotheses; }
    const std::vector<ObjectHypothesis>& getRatedPenatlySpotHypotheses() const { return ratedPenaltySpotHypotheses; }
    const std::vector<ObjectHypothesis>& getFeetHypotheses() const { return ratedFeetHypotheses; }
};

}  // namespace htwk

#endif  // OBJECT_DETECTOR_H
