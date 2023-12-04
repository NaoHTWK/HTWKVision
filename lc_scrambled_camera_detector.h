#pragma once

#include <base_detector.h>
#include <htwk_vision_config.h>
#include <image_preprocessor.h>
#include <tfliteexecuter.h>
#include <field_color_detector.h>

#include <memory>
#include <vector>

namespace htwk {

class LowerCameraScrambledCameraDetector : BaseDetector {
public:
    const int imageWidth;
    const int imageHeight;

    LowerCameraScrambledCameraDetector(int8_t* lutCb, int8_t* lutCr, HtwkVisionConfig& config);

    LowerCameraScrambledCameraDetector(const LowerCameraScrambledCameraDetector&) = delete;
    LowerCameraScrambledCameraDetector(const LowerCameraScrambledCameraDetector&&) = delete;
    LowerCameraScrambledCameraDetector& operator=(const LowerCameraScrambledCameraDetector&) = delete;
    LowerCameraScrambledCameraDetector& operator=(LowerCameraScrambledCameraDetector&&) = delete;
    ~LowerCameraScrambledCameraDetector();

    void proceed(std::shared_ptr<ImagePreprocessor> imagePreprocessor);
    bool isCameraScrambled() {
        return isCameraScrambledFlag;
    }

    bool wasExecuted() {
        return wasExecutedFlag;
    }

    void shouldRun(bool state);

private:
    static constexpr int channels = 3;
    bool isCameraScrambledFlag = false;
    bool wasExecutedFlag;

    float* input;
    TFLiteExecuter modelExecutor;
};

}  // namespace htwk
