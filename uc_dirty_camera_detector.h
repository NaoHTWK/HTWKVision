#pragma once

#include <base_detector.h>
#include <htwk_vision_config.h>
#include <image_preprocessor.h>
#include <tfliteexecuter.h>
#include <field_color_detector.h>

#include <memory>
#include <vector>

namespace htwk {

class UpperCamDirtyCameraDetector : BaseDetector {
public:
    const int imageWidth;
    const int imageHeight;

    UpperCamDirtyCameraDetector(int8_t* lutCb, int8_t* lutCr, HtwkVisionConfig& config, FieldColorDetector* field_color_detector);

    UpperCamDirtyCameraDetector(const UpperCamDirtyCameraDetector&) = delete;
    UpperCamDirtyCameraDetector(const UpperCamDirtyCameraDetector&&) = delete;
    UpperCamDirtyCameraDetector& operator=(const UpperCamDirtyCameraDetector&) = delete;
    UpperCamDirtyCameraDetector& operator=(UpperCamDirtyCameraDetector&&) = delete;
    ~UpperCamDirtyCameraDetector();

    void proceed(const uint8_t* img, std::shared_ptr<ImagePreprocessor> imagePreprocessor);
    bool isCameraDirty() {
        return isCameraDirtyFlag;
    }

    bool wasExecuted() {
        return wasExecutedFlag;
    }

    void shouldRun(bool state);

private:
    static constexpr int channels = 3;
    bool isCameraDirtyFlag = false;
    bool wasExecutedFlag;

    float* input;
    TFLiteExecuter modelExecutor;
    FieldColorDetector* fieldColorDetector;
};

}  // namespace htwk
