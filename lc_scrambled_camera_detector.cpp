#include "lc_scrambled_camera_detector.h"

#include <algorithm_ext.h>
#include <easy/profiler.h>
#include <line.h>
#include <stl_ext.h>

#include <cstring>

using namespace std;

namespace htwk {

LowerCameraScrambledCameraDetector::LowerCameraScrambledCameraDetector(int8_t *lutCb, int8_t *lutCr,
                                                                       HtwkVisionConfig &config)
    : BaseDetector(lutCb, lutCr, config),
      imageWidth(config.lcObjectDetectorConfig.scaledImageWidth),
      imageHeight(config.lcObjectDetectorConfig.scaledImageHeight) {
    const auto &conf = config.lcScrambledCameraDetector;

    modelExecutor.loadModelFromFile(config.tflitePath + "/" + conf.model, {1, imageHeight, imageWidth, channels},
                                    conf.threads);
    input = modelExecutor.getInputTensor();

    if (config.lcObjectDetectorConfig.scaledImageWidth != config.lcScrambledCameraDetector.scaledImageWidth ||
        config.lcObjectDetectorConfig.scaledImageHeight != config.lcScrambledCameraDetector.scaledImageHeight) {
        printf("%s:%d - scaled image does not have matching sizes width (%d vs %d), height (%d vs %d\n)", __FILE__,
               __LINE__, config.lcObjectDetectorConfig.scaledImageWidth,
               config.lcScrambledCameraDetector.scaledImageHeight, config.lcObjectDetectorConfig.scaledImageHeight,
               config.lcScrambledCameraDetector.scaledImageHeight);
        fflush(stdout);
        exit(1);
    }
}

LowerCameraScrambledCameraDetector::~LowerCameraScrambledCameraDetector() {}

void LowerCameraScrambledCameraDetector::proceed(std::shared_ptr<ImagePreprocessor> imagePreprocessor) {
    Timer t("LowerCameraScrambledCameraDetector", 100);
    EASY_FUNCTION();

    wasExecutedFlag = false;

    if (!config.lcScrambledCameraDetector.run)
        return;

    wasExecutedFlag = true;
    EASY_BLOCK("LowerCameraScrambledCameraDetector Prepare");
    std::memcpy(input, imagePreprocessor->getScaledImage().data(),
                imagePreprocessor->getScaledImage().size() * sizeof(float));
    EASY_END_BLOCK;

    EASY_BLOCK("LowerCameraScrambledCameraDetector Execute");
    modelExecutor.execute();
    EASY_END_BLOCK;

    isCameraScrambledFlag = *modelExecutor.getOutputTensor() > 0.99;
}

void LowerCameraScrambledCameraDetector::shouldRun(bool state) {
    config.lcScrambledCameraDetector.run = state;
}

}  // namespace htwk
