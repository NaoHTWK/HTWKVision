#pragma once

#include <base_detector.h>
#include <htwk_vision_config.h>
#include <image_preprocessor.h>
#include <localization_utils.h>
#include <object_hypothesis.h>
#include <tfliteexecuter.h>

#include <async.h>
#include <memory>
#include <vector>

namespace htwk {

class UpperCamBallHypothesesGenerator : BaseDetector {
public:
    const int patchWidth;
    const int patchHeight;
    const int imageWidth;
    const int imageHeight;

    UpperCamBallHypothesesGenerator(int8_t* lutCb, int8_t* lutCr, HtwkVisionConfig& config, ThreadPool* thread_pool);

    UpperCamBallHypothesesGenerator(const UpperCamBallHypothesesGenerator&) = delete;
    UpperCamBallHypothesesGenerator(const UpperCamBallHypothesesGenerator&&) = delete;
    UpperCamBallHypothesesGenerator& operator=(const UpperCamBallHypothesesGenerator&) = delete;
    UpperCamBallHypothesesGenerator& operator=(UpperCamBallHypothesesGenerator&&) = delete;
    ~UpperCamBallHypothesesGenerator();

    void proceed(CamPose& cam_pose, std::shared_ptr<ImagePreprocessor> imagePreprocessor);

    std::vector<ObjectHypothesis> getHypotheses() const {
        return hypotheses;
    }

    void drawPatch(uint8_t* yuvImage, int px, int py);

private:
    static constexpr int channels = 3;
    static constexpr int num_threads = 4;

    std::vector<ObjectHypothesis> hypotheses;
    float* inputHypFinder[num_threads];
    TFLiteExecuter hypGenExecuter[num_threads];
    ThreadPool* thread_pool;

    constexpr float scale(float x, float from_min, float from_max, float to_min, float to_max) {
        return ((to_max - to_min) * (x - from_min)) / (from_max - from_min) + to_min;
    }

    constexpr float unscale_x(float x) {
        return scale(x, -1, 1, -patchWidth / 2 * 1.2, patchWidth / 2 * 1.2);
    }

    constexpr float unscale_y(float y) {
        return scale(y, -1, 1, -patchHeight / 2 * 1.2, patchHeight / 2 * 1.2);
    }
};

}  // namespace htwk
