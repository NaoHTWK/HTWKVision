#include "uc_centercirclepoints_detector.h"

#include <algorithm_ext.h>
#include <easy/profiler.h>
#include <line.h>
#include <stl_ext.h>

#include <cstring>

using namespace std;

namespace htwk {

UpperCamCenterCirclePointDetector::UpperCamCenterCirclePointDetector(int8_t* lutCb, int8_t* lutCr, HtwkVisionConfig &config)
    : BaseDetector(lutCb, lutCr, config),
      imageWidth(config.ucCenterCirclePointDetectorConfig.scaledImageWidth),
      imageHeight(config.ucCenterCirclePointDetectorConfig.scaledImageHeight),
      patchesX(config.ucCenterCirclePointDetectorConfig.patchesX),
      patchesY(config.ucCenterCirclePointDetectorConfig.patchesY),
      patchWidth(imageWidth / patchesX),
      patchHeight(imageHeight / patchesY) {
    const int hypCount = patchesX * patchesY;

    const auto& gdConf = config.ucCenterCirclePointDetectorConfig;
    if (gdConf.runDetector) {
        detector.loadModelFromFile(config.tflitePath + "/" + gdConf.model,
                                   {hypCount, patchHeight, patchWidth, channels}, gdConf.threads);
        inputDetector = detector.getInputTensor();
    } else {
        size_t alloc_size = hypCount * patchWidth * patchHeight * channels;
        // aligned_alloc expects multiple of the alignment size as size.
        inputDetector = (float*)aligned_alloc(16, ((alloc_size * sizeof(float) + 15) / 16) * 16);

        if (inputDetector == nullptr) {
            fprintf(stdout, "%s:%d: %s error allocation input array!\n", __FILE__, __LINE__, __func__);
            exit(1);
        }
    }
}

UpperCamCenterCirclePointDetector::~UpperCamCenterCirclePointDetector() {
    if (!config.ucCenterCirclePointDetectorConfig.runDetector) {
        free(inputDetector);
    }
}

void UpperCamCenterCirclePointDetector::proceed(CamPose& cam_pose, std::shared_ptr<ImagePreprocessor> imagePreprocessor) {
    Timer t("UpperCamCenterCirclePointDetector", 50);
    EASY_FUNCTION();
    const float* img = imagePreprocessor->getScaledImage().data();

    EASY_BLOCK("UpperCamCenterCirclePointDetector Prepare");
    for (int py = 0; py < patchesY; py++) {
        for (int px = 0; px < patchesX; px++) {
            float* output = inputDetector + (py * patchesX + px) * (patchWidth * patchHeight * 3);

            int iy = py * patchHeight * (patchesX * patchWidth * 3);
            int ix = px * (patchWidth * 3);

            // printf("y: %d, x: %d = %d\n", py, px, (py * patches_x + px) * (patchWidth * patchHeight * 3));
            for (int y = 0; y < patchHeight; y++) {
                for (int x = 0; x < patchWidth; x++) {
                    output[0 + x * 3 + y * (3 * patchWidth)] =
                            img[0 + (ix + x * 3) + (iy + y * (3 * patchWidth * patchesX))];
                    output[1 + x * 3 + y * (3 * patchWidth)] =
                            img[1 + (ix + x * 3) + (iy + y * (3 * patchWidth * patchesX))];
                    output[2 + x * 3 + y * (3 * patchWidth)] =
                            img[2 + (ix + x * 3) + (iy + y * (3 * patchWidth * patchesX))];
                    // printf("pyx(%d, %d), yx(%d, %d), Y%0.2f, o(%d), i(%d)\n", py, px, y, x, output[0 + x * 3 + y * (3
                    // * patchWidth)], 0 + x * 3 + y * (3 * patchWidth), (ix + x * 3) + (iy + y * (3 * patchWidth *
                    // patches_x)));
                }
            }
        }
    }
    EASY_END_BLOCK;

    if (!config.ucCenterCirclePointDetectorConfig.runDetector)
        return;

    EASY_BLOCK("UpperCamCenterCirclePointDetector Hyp");
    detector.execute();
    EASY_END_BLOCK;

    hypotheses.clear();
    const float* result = detector.getOutputTensor();
    const int outputsPerPatch = 3;

    for (int y = 0; y < patchesY; y++) {
        for (int x = 0; x < patchesX; x++) {
            const float* res = result + (y * patchesX + x) * outputsPerPatch;

            if (res[0] < config.ucCenterCirclePointDetectorConfig.probabilityThreshold)
                continue;

            ObjectHypothesis h;
            float ux = unscale_x(res[1]) + patchWidth / 2;  // position in patch
            float ox = ux + (x * patchWidth);               // position in downscaled image
            h.x = ox * (width / imageWidth);                // position in big image

            float uy = (unscale_y(res[2]) + patchHeight / 2);
            float oy = uy + (y * patchHeight);
            h.y = oy * (height / imageHeight);

            auto radius = LocalizationUtils::getPixelRadius(h, cam_pose, 0.05f);
            h.r = radius ? *radius : -1000;

            h.prob = res[0];
            hypotheses.push_back(h);
        }
    }
}

void UpperCamCenterCirclePointDetector::drawPatch(uint8_t* yuvImage, int px, int py) {
    const int blockSizeWidth = width / patchWidth;
    const int blockSizeHeight = height / patchHeight;
    const int channels = 3;

    const int patches_x = imageWidth / patchWidth;
    float* img = inputDetector + (py * patches_x + px) * (patchWidth * patchHeight * 3);

    for (int ySample = 0; ySample < patchHeight; ySample++) {
        const int startY = ySample * blockSizeHeight;

        for (int xSample = 0; xSample < patchWidth; xSample++) {
            const int startX = xSample * blockSizeWidth;

            const int vcy = (int)(img[0 + channels * xSample + ySample * channels * patchWidth] * 255.f);
            const int vcb = (int)(img[1 + channels * xSample + ySample * channels * patchWidth] * 255.f);
            const int vcr = (int)(img[2 + channels * xSample + ySample * channels * patchWidth] * 255.f);
            const uint8_t cy = static_cast<uint8_t>(std::min(std::max(0, vcy), 255));
            const uint8_t cb = static_cast<uint8_t>(std::min(std::max(0, vcb), 255));
            const uint8_t cr = static_cast<uint8_t>(std::min(std::max(0, vcr), 255));

            for (int y = startY; y < startY + blockSizeHeight; y++) {
                for (int x = startX; x < startX + blockSizeWidth; x++) {
                    setYCbCr(yuvImage, x, y, cy, cb, cr);
                }
            }
        }
    }
}

}  // namespace htwk
