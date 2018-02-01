#ifndef BALL_DETECTOR_LEGACY_H
#define BALL_DETECTOR_LEGACY_H

#include <vector>
#include <cstddef>
#include <cstdint>

#include <ball_detector.h>
#include <ball_feature_extractor.h>
#include <base_detector.h>
#include <htwk_vision_config.h>
#include <object_hypothesis.h>

#include <Eigen/Dense>
#include <classifier_relu.h>

namespace htwk {

class BallDetectorLegacy : public BallDetector {

private:
    static constexpr int FEATURE_SIZE=18;
    static constexpr int FEATURE_SIZE_PRE=8;
    static constexpr int NUM_EVALUATIONS=1;
	const float MIN_PROB;
    bool found;
    BallFeatureExtractor* featureExtractor;
    ObjectHypothesis bestHypothesis;

    std::vector<ObjectHypothesis> ratedHypothesis;

    ClassifierReLU* ballClassifierPre;
    ClassifierReLU* ballClassifier;

    Eigen::MatrixXf getFeature(const ObjectHypothesis &p, const uint8_t *img, const int size);

public:
    BallDetectorLegacy(const int width, const int height,
                       const int8_t* lutCb, const int8_t* lutCr,
                       BallFeatureExtractor* _featureExtractor, const HtwkVisionConfig& config) __attribute__((nonnull));

    ~BallDetectorLegacy() override;

    void proceed(const uint8_t *img, std::vector<ObjectHypothesis> &hypoList) override;
    bool isBallFound() const override { return found; }
    const ObjectHypothesis& getBall() const override { return bestHypothesis; }

    // This is used by the machine learning tools. This must not be used in the firmware!
    const std::vector<ObjectHypothesis>& getRatedHypothesis() const { return ratedHypothesis; }
};

}  // namespace htwk

#endif  // BALL_DETECTOR_LEGACY_H
