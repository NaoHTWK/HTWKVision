#ifndef ROBOT_DETECTOR_H
#define ROBOT_DETECTOR_H

#include <cstdint>
#include <vector>

#include <base_detector.h>
#include "ball_feature_extractor.h"
#include "htwk_vision_config.h"
#include <object_hypothesis.h>

#include <caffe/caffe.hpp>

namespace htwk{
class RobotDetector : protected BaseDetector {

private:
    const float MIN_FEET_PROB;
    const int FEATURE_SIZE;

    int foundFeet;
    std::vector<ObjectHypothesis> ratedHypothesis;

    BallFeatureExtractor* featureExtractor;

    std::shared_ptr<caffe::Net<float>> classifier;
    caffe::Blob<float>* inputLayer;
    caffe::Blob<float>* outputLayer;

public:

    RobotDetector(int width, int height, int8_t *lutCb, int8_t *lutCr, BallFeatureExtractor *_featureExtractor, const HtwkVisionConfig &config) __attribute__((nonnull));
    void proceed(uint8_t *img, std::vector<ObjectHypothesis> &hypoList) __attribute__((nonnull));

    int feetsFound() const { return foundFeet; }

    // This is used by the machine learning tools. This must not be used in the firmware!
    const std::vector<ObjectHypothesis>& getFeetHypotheses() const { return ratedHypothesis; }
};

}  // namespace htwk

#endif // ROBOT_DETECTOR_H
