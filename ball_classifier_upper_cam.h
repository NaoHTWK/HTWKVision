#pragma once

#include <vector>

#include <ball_detector.h>
#include <base_detector.h>
#include <htwk_vision_config.h>
#include <hypotheses_generator.h>
#include <localization_utils.h>
#include <object_hypothesis.h>
#include <point_2d.h>
#include <tfliteexecuter.h>

namespace htwk {

class BallClassifierUpperCam : public BallDetector {
public:
    const int hypo_size_x;
    const int hypo_size_y;

    BallClassifierUpperCam(int8_t* lutCb, int8_t* lutCr, HtwkVisionConfig& config);
    BallClassifierUpperCam(const BallClassifierUpperCam&) = delete;
    BallClassifierUpperCam(const BallClassifierUpperCam&&) = delete;
    BallClassifierUpperCam& operator=(const BallClassifierUpperCam&) = delete;
    BallClassifierUpperCam& operator=(BallClassifierUpperCam&&) = delete;
    ~BallClassifierUpperCam() override;

    void proceed(uint8_t* img, std::vector<ObjectHypothesis>& hypotheses, CamPose& cam_pose);

    struct Augmentation {
        point_2d translation;  //!< translation of center in x and y in percent.
        float rotation;
        float scale;  //!< normal size is 1
        bool mirror;
    };
    std::vector<float> generateAugmentedHypothesis(uint8_t* img, const ObjectHypothesis& obj_hyp, CamPose& cam_pose, Augmentation aug);

    //    void drawInputParameter(uint8_t* yuvImage);

    /**
     * You don't want to call this! This is the ball hyp without classification
     */
    const std::vector<ObjectHypothesis>& getRatedBallHypotheses() const override {
        return ratedBallHypothesis;
    }

    const std::vector<ObjectHypothesis>& getAllHypothesesWithProb() const override {
        return allRatedHypothesis;
    }

    /**
     * This is the current ball if a ball exists.
     */
    const std::optional<ObjectHypothesis>& getBall() const override {
        return ballClassifierResult;
    }

private:
    TFLiteExecuter tflite;
    float* classifierInput = nullptr;
    const size_t num_hypotheses_to_test;
    const float smallBallProbThreshold;

    bool shouldWeClassifyBallData;
    std::vector<ObjectHypothesis> ratedBallHypothesis;
    std::vector<ObjectHypothesis> allRatedHypothesis;
    std::optional<ObjectHypothesis> ballClassifierResult;

    static constexpr int channels = 3;

    void createInputData(uint8_t* img);
    void generateHypothesis(uint8_t* img, CamPose& cam_pose, ObjectHypothesis& hyp, size_t offset);
};

}  // namespace htwk
