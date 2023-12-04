#pragma once

#include <base_detector.h>
#include <htwk_vision_config.h>
#include <image_preprocessor.h>
#include <localization_utils.h>
#include <object_hypothesis.h>
#include <tfliteexecuter.h>

#include <cstring>
#include <memory>
#include <vector>

namespace htwk {

class ObjectDetectorLowCamHypGen : BaseDetector {
public:
    const int inputWidth;
    const int inputHeight;

    ObjectDetectorLowCamHypGen(int8_t* lutCb, int8_t* lutCr, HtwkVisionConfig& config, std::string modelFile,
                               std::shared_ptr<ImagePreprocessor> imagePreprocessor);
    ObjectDetectorLowCamHypGen(const ObjectDetectorLowCamHypGen&) = delete;
    ObjectDetectorLowCamHypGen(const ObjectDetectorLowCamHypGen&&) = delete;
    ObjectDetectorLowCamHypGen& operator=(const ObjectDetectorLowCamHypGen&) = delete;
    ObjectDetectorLowCamHypGen& operator=(ObjectDetectorLowCamHypGen&&) = delete;
    ~ObjectDetectorLowCamHypGen();

    void proceed(CamPose& cam_pose);

    ObjectHypothesis getObjectHypotheses() const {
        return outputObject;
    }

private:
    static constexpr int channels = 3;
    std::shared_ptr<ImagePreprocessor> imagePreprocessor;

    ObjectHypothesis outputObject;
    float* inputHypFinder;

    TFLiteExecuter hypGenExecuter;

    constexpr float scale(float x, float from_min, float from_max, float to_min, float to_max) {
        return ((to_max - to_min) * (x - from_min)) / (from_max - from_min) + to_min;
    }

    constexpr float unscale_x(float x) {
        return scale(x, -1, 1, -inputWidth / 2 * 1.2, inputWidth / 2 * 1.2);
    }

    constexpr float unscale_y(float y) {
        return scale(y, -1, 1, -inputHeight / 2 * 1.2, inputHeight / 2 * 1.2);
    }
};

}  // namespace htwk
