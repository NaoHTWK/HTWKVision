#pragma once

#include <cstring>
#include <vector>

#include <base_detector.h>
#include <ball_detector.h>
#include <htwk_vision_config.h>
#include <localization_utils.h>
#include <object_hypothesis.h>
#include <point_2d.h>

#include <tfliteexecuter.h>

namespace htwk {

class ObjectDetectorLowCam : public BallDetector {
public:
    const int inputWidth;
    const int inputHeight;
    const int hypo_size_x;
    const int hypo_size_y;

    ObjectDetectorLowCam(int8_t* lutCb, int8_t* lutCr, HtwkVisionConfig& config);
    ObjectDetectorLowCam(const ObjectDetectorLowCam&) = delete;
    ObjectDetectorLowCam(const ObjectDetectorLowCam&&) = delete;
    ObjectDetectorLowCam& operator=(const ObjectDetectorLowCam&) = delete;
    ObjectDetectorLowCam& operator=(ObjectDetectorLowCam&&) = delete;
    ~ObjectDetectorLowCam() override;

    void proceed(uint8_t* img, CamPose& cam_pose, ObjectHypothesis inBallHyp, ObjectHypothesis inPenaltySpotHyp);

    struct Augmentation {
        point_2d translation; //!< translation of center in x and y in percent.
        float rotation;
        float scale; //!< normal size is 1
        bool mirror;
    };
    std::vector<float> generateAugmentedHypothesis(uint8_t* img, CamPose& cam_pose, const ObjectHypothesis& hypPos, Augmentation aug);

    /**
     * You don't want to call this! This is the ball hyp without classification
     */
    const std::vector<ObjectHypothesis>& getRatedBallHypotheses() const override {
        return ballHypothesis;
    }

    const std::vector<ObjectHypothesis>& getAllHypothesesWithProb() const override {
        return ballHypothesis;
    }

    /**
     * You don't want to call this! This is the penaltyspot hyp without classification
     */
    const std::vector<ObjectHypothesis>& getRatedPenaltySpotHypotheses() const {
        return penaltySpotHypothesis;
    }

    /**
     * This is the current ball if a ball exists.
     */
    const std::optional<ObjectHypothesis>& getBall() const override {
        return ballClassifierResult;
    }

    /**
     * This is the current penaltyspot if a penaltyspot exists.
     */
    const std::optional<ObjectHypothesis>& getPenaltySpot() const {
        return penaltySpotClassifierResult;
    }

private:
    point_2d outputBall;
    point_2d outputPenaltySpot;

    std::vector<ObjectHypothesis> ballHypothesis;
    std::vector<ObjectHypothesis> penaltySpotHypothesis;

    std::optional<ObjectHypothesis> ballClassifierResult;
    std::optional<ObjectHypothesis> penaltySpotClassifierResult;

    static constexpr int channels = 3;
    float* inputHypClassifier;

    void generateHypothesis(uint8_t* img, CamPose& cam_pose, const ObjectHypothesis &hypPos, float *output);

    TFLiteExecuter hypClassifierExecuter;
};

}  // namespace htwk
