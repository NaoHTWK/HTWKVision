#include "uc_dirty_camera_detector.h"

#include <algorithm_ext.h>
#include <easy/profiler.h>
#include <line.h>
#include <stl_ext.h>

#include <cstring>

using namespace std;

namespace htwk {

UpperCamDirtyCameraDetector::UpperCamDirtyCameraDetector(int8_t *lutCb, int8_t *lutCr, HtwkVisionConfig &config,
                                                         FieldColorDetector *field_color_detector)
    : BaseDetector(lutCb, lutCr, config),
      imageWidth(config.ucBallHypGeneratorConfig.scaledImageWidth),
      imageHeight(config.ucBallHypGeneratorConfig.scaledImageHeight),
      fieldColorDetector(field_color_detector) {
    const auto &conf = config.ucDirtyCameraDetector;

    modelExecutor.loadModelFromFile(config.tflitePath + "/" + conf.model, {1, imageHeight, imageWidth, channels},
                                    conf.threads);
    input = modelExecutor.getInputTensor();

    if (config.ucBallHypGeneratorConfig.scaledImageWidth != config.ucDirtyCameraDetector.scaledImageWidth ||
        config.ucBallHypGeneratorConfig.scaledImageHeight != config.ucDirtyCameraDetector.scaledImageHeight) {
        printf("%s:%d - scaled image does not have matching sizes width (%d vs %d), height (%d vs %d)\n", __FILE__,
               __LINE__, config.lcObjectDetectorConfig.scaledImageWidth,
               config.lcScrambledCameraDetector.scaledImageHeight, config.lcObjectDetectorConfig.scaledImageHeight,
               config.lcScrambledCameraDetector.scaledImageHeight);
        fflush(stdout);
        exit(1);
    }
}

UpperCamDirtyCameraDetector::~UpperCamDirtyCameraDetector() {}

void UpperCamDirtyCameraDetector::proceed(const uint8_t *img, std::shared_ptr<ImagePreprocessor> imagePreprocessor) {
    Timer t("UpperCamDirtyCameraDetector", 150);
    EASY_FUNCTION();

    wasExecutedFlag = false;

    if (!config.ucDirtyCameraDetector.run)
        return;

    int testedPixelCount = 0;
    int greenPixelCount = 0;
    for (int y = config.height / 4; y < config.height; y += config.height / 10) {
        for (int x = 32; x < config.width - 32; x += config.width / 10) {
            testedPixelCount++;
            greenPixelCount += fieldColorDetector->isGreen(img, x, y) ? 1 : 0;
        }
    }

    float greenPixelPercentage = greenPixelCount / (float)testedPixelCount;
    if (greenPixelPercentage < 0.4)
        return;

    wasExecutedFlag = true;
    EASY_BLOCK("UpperCamDirtyCameraDetector Prepare");
    std::memcpy(input, imagePreprocessor->getScaledImage().data(),
                imagePreprocessor->getScaledImage().size() * sizeof(float));
    EASY_END_BLOCK;

    EASY_BLOCK("UpperCamDirtyCameraDetector Hyp");
    modelExecutor.execute();
    EASY_END_BLOCK;

    isCameraDirtyFlag = *modelExecutor.getOutputTensor() < 0.03;
}

void UpperCamDirtyCameraDetector::shouldRun(bool state) {
    config.ucDirtyCameraDetector.run = state;
}

}  // namespace htwk
