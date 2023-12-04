#include "lc_centercirclepoints_detector.h"

#include <algorithm_ext.h>
#include <easy/profiler.h>
#include <line.h>
#include <stl_ext.h>

#include <cstring>

using namespace std;

namespace htwk {

LowerCamCenterCirclePointDetector::LowerCamCenterCirclePointDetector(int8_t* lutCb, int8_t* lutCr,
                                                                     HtwkVisionConfig &config, DetectorType type)
    : BaseDetector(lutCb, lutCr, config),
      imageWidth(config.lcCenterCirclePointDetectorConfig.scaledImageWidth),
      imageHeight(config.lcCenterCirclePointDetectorConfig.scaledImageHeight),
      detector_type(type) {

    const int hypCount = 1;

    const auto& gdConf = config.lcCenterCirclePointDetectorConfig;
    const auto model = type == CENTER ? gdConf.model_center : gdConf.model_side;
    if (gdConf.runDetector) {
        detector.loadModelFromFile(config.tflitePath + "/" + model, {hypCount, imageHeight, imageWidth, channels}, 1);
        inputDetector = detector.getInputTensor();
    } else {
        size_t alloc_size = hypCount * channels;
        // aligned_alloc expects multiple of the alignment size as size.
        inputDetector = (float*)aligned_alloc(16, ((alloc_size * sizeof(float) + 15) / 16) * 16);

        if (inputDetector == nullptr) {
            fprintf(stdout, "%s:%d: %s error allocation input array!\n", __FILE__, __LINE__, __func__);
            exit(1);
        }
    }
}

LowerCamCenterCirclePointDetector::~LowerCamCenterCirclePointDetector() {
    if (!config.ucCenterCirclePointDetectorConfig.runDetector) {
        free(inputDetector);
    }
}

ObjectHypothesis LowerCamCenterCirclePointDetector::resultToHyp(const float* result, CamPose& cam_pose, size_t clazz,
                                                                size_t x, size_t y) {
    ObjectHypothesis h;
    h.prob = result[clazz];
    h.x = (unscale_x(result[x]) + imageWidth / 2) * (width / imageWidth);
    h.y = (unscale_y(result[y]) + imageHeight / 2) * (height / imageHeight);

    auto radius = LocalizationUtils::getPixelRadius(h, cam_pose, 0.05f);
    h.r = radius ? *radius : -1000;
    return h;
}

void LowerCamCenterCirclePointDetector::proceed(CamPose& cam_pose,
                                                std::shared_ptr<ImagePreprocessor> imagePreprocessor) {
    Timer t("LowerCamCenterCirclePointDetector", 50);
    EASY_FUNCTION();

    EASY_BLOCK("LowerCamCenterCirclePointDetector Prepare");
    std::memcpy(inputDetector, imagePreprocessor->getScaledImage().data(),
                imagePreprocessor->getScaledImage().size() * sizeof(float));
    EASY_END_BLOCK;

    if (!config.lcCenterCirclePointDetectorConfig.runDetector)
        return;

    EASY_BLOCK("LowerCamCenterCirclePointDetector Hyp");
    detector.execute();
    EASY_END_BLOCK;

    const float* result = detector.getOutputTensor();

    hypotheses.clear();
    const auto& cfg = config.lcCenterCirclePointDetectorConfig;
    float threshold = detector_type == CENTER ? cfg.probabilityThresholdCenter : cfg.probabilityThresholdSide;

    ObjectHypothesis tmp_hyp = resultToHyp(result, cam_pose, 1, 2, 3);
    hypotheses.push_back(tmp_hyp);

    cur_hyp = std::nullopt;
    if (result[1] > threshold) {
        cur_hyp = tmp_hyp;
    }
}

}  // namespace htwk
