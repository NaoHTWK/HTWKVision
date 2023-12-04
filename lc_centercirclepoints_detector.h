#pragma once

#include <base_detector.h>
#include <htwk_vision_config.h>
#include <image_preprocessor.h>
#include <localization_utils.h>
#include <object_hypothesis.h>
#include <tfliteexecuter.h>

#include <memory>
#include <optional>
#include <vector>

namespace htwk {

class LowerCamCenterCirclePointDetector : BaseDetector {
public:
    enum DetectorType { CENTER, SIDE };

    LowerCamCenterCirclePointDetector(int8_t* lutCb, int8_t* lutCr, HtwkVisionConfig& config, DetectorType type);

    LowerCamCenterCirclePointDetector(const LowerCamCenterCirclePointDetector&) = delete;
    LowerCamCenterCirclePointDetector(const LowerCamCenterCirclePointDetector&&) = delete;
    LowerCamCenterCirclePointDetector& operator=(const LowerCamCenterCirclePointDetector&) = delete;
    LowerCamCenterCirclePointDetector& operator=(LowerCamCenterCirclePointDetector&&) = delete;
    ~LowerCamCenterCirclePointDetector();

    void proceed(CamPose& cam_pose, std::shared_ptr<ImagePreprocessor> imagePreprocessor);

    std::optional<ObjectHypothesis> getHypothesis() {
        return cur_hyp;
    }

    std::vector<ObjectHypothesis> getHypotheses() const {
        return hypotheses;
    }

private:
    static constexpr int channels = 3;

    const int imageWidth;
    const int imageHeight;

    DetectorType detector_type;

    std::optional<ObjectHypothesis> cur_hyp;
    std::vector<ObjectHypothesis> hypotheses;

    float* inputDetector;

    TFLiteExecuter detector;

    constexpr float scale(float x, float from_min, float from_max, float to_min, float to_max) {
        return ((to_max - to_min) * (x - from_min)) / (from_max - from_min) + to_min;
    }

    constexpr float unscale_x(float x) {
        return scale(x, -1, 1, -imageWidth / 2 * 1.2, imageWidth / 2 * 1.2);
    }

    constexpr float unscale_y(float y) {
        return scale(y, -1, 1, -imageHeight / 2 * 1.2, imageHeight / 2 * 1.2);
    }

    ObjectHypothesis resultToHyp(const float* result, CamPose& cam_pose, size_t clazz, size_t x, size_t y);
};

}  // namespace htwk
