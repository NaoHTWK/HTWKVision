#include "lc_obstacle_detection.h"

#include <cstring>

#include <easy/profiler.h>
#include <emmintrin.h>

#include <algorithm_ext.h>
#include <stl_ext.h>

namespace htwk {

LowerCamObstacleDetection::LowerCamObstacleDetection(int8_t* lutCb, int8_t* lutCr, HtwkVisionConfig &config)
    : BaseDetector(lutCb, lutCr, config),
      inputHeight(config.obstacleDetectionInputHeight),
      inputWidth(config.obstacleDetectionInputWidth),
      inputSize(inputHeight * inputWidth * 3),
      detectionResult(outputWidth * outputHeight),
      shouldWeClassifyData(config.obstacleClassifyData),
      isOutputValid(false) {

    if (shouldWeClassifyData) {
        tflite.loadModelFromFile(config.tflitePath + "/lc-obstacle-classifier.tflite", {1, inputHeight, inputWidth, 3});
        input = tflite.getInputTensor();
    } else {
        // aligned_alloc expects multiple of the alignment size as size.
        input = (float*)aligned_alloc(16, ((inputSize * sizeof(float) + 15) / 16) * 16);

        if (input == nullptr) {
            fprintf(stdout, "%s:%d: %s error allocation xlaInput array!", __FILE__, __LINE__, __func__);
            exit(1);
        }
    }
}

LowerCamObstacleDetection::~LowerCamObstacleDetection() {
    // We only have to free data when we don't use tflite
    if(!config.obstacleClassifyData)
        free(input);
}

bool LowerCamObstacleDetection::isShoulderInImage(float headYaw) {
    const float weight = 5.38412472f;
    const float bias = -4.4397172f;

    return 1.f / (1.f + std::exp(-1.f * (std::fabs(headYaw) * weight) - bias)) > 0.25f;
}

void LowerCamObstacleDetection::drawInputParameter(uint8_t* yuvImage) {
    const int blockSizeWidth = width / inputWidth;
    const int blockSizeHeight = height / inputHeight;
    int channels = 3;

    for (int ySample = 0; ySample < inputHeight; ySample++) {
        const int startY = ySample * blockSizeHeight;

        for (int xSample = 0; xSample < inputWidth; xSample++) {
            const int startX = xSample * blockSizeWidth;

            const int vcy = (int)((input[0 + channels*xSample + ySample * channels*inputWidth] + 1.f)*128.f);
            const int vcb = (int)((input[1 + channels*xSample + ySample * channels*inputWidth] + 1.f)*128.f);
            const int vcr = (int)((input[2 + channels*xSample + ySample * channels*inputWidth] + 1.f)*128.f);
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

void LowerCamObstacleDetection::drawResult(uint8_t* yuvImage) {
    const int blockSizeWidth = width / outputWidth;
    const int blockSizeHeight = height / outputHeight;

    for (int ySample = 0; ySample < outputHeight; ySample++) {
        const int startY = ySample * blockSizeHeight;

        for (int xSample = 0; xSample < outputWidth; xSample++) {
            const int startX = xSample * blockSizeWidth;

            const int brightnessValue = (int)(detectionResult[xSample + ySample * outputWidth] * 255.f);
            const uint8_t brightness = static_cast<uint8_t>(std::min(std::max(0, brightnessValue), 255));

            for (int y = startY; y < startY + blockSizeHeight; y++) {
                for (int x = startX; x < startX + blockSizeWidth; x++) {
                    setY(yuvImage, x, y, brightness);
                }
            }
        }
    }
}

void LowerCamObstacleDetection::proceed(float headYaw, uint8_t* img) {
    Timer t("ObstacleDetectionLowCam", 50);
    EASY_FUNCTION();

    isOutputValid = false;

    // Don't process image if it's a lost cause.
    if (isShoulderInImage(headYaw)) {
        std::fill(detectionResult.begin(), detectionResult.end(), 0.f);
        return;
    }

    EASY_BLOCK("ObstacleDetector Inputdata");
    createInputData(img);
    EASY_END_BLOCK;

    if (shouldWeClassifyData) {
        isOutputValid = true;
        classifyData();
    }
}

std::vector<float> LowerCamObstacleDetection::getInputParameter() {
    std::vector<float> v(inputSize);
    memcpy(v.data(), input, v.size() * sizeof(float));
    return v;
}

void LowerCamObstacleDetection::classifyData() {
    EASY_BLOCK("ObstacleDetector Classifier");
    tflite.execute();
    EASY_END_BLOCK;

    const float* outputPos = tflite.getOutputTensor();

    for (size_t i = 0; i < detectionResult.size(); i++)
        detectionResult[i] = outputPos[i];
}

void LowerCamObstacleDetection::createInputData(uint8_t* img) {
    const int blockWidth = width / inputWidth;
    const int blockHeight = height / inputHeight;
    float fac = 1.f / (128.f * (blockWidth * blockHeight));
    float fac2 = 1.f / (128.f * (blockWidth * blockHeight / 2));

    for (int y = 0; y < inputHeight; y++) {
        for (int x = 0; x < inputWidth; x++) {
            int valY = 0;
            int valU = 0;
            int valV = 0;
            for (int y2 = 0; y2 < blockHeight; y2++) {
                // yy uu yy vv yy uu yy vv
                __m128i sum16 = _mm_setzero_si128();
                uint8_t* start = img + x * blockWidth * 2 + (y * blockHeight + y2) * width * 2;
                __m128i in0 = _mm_loadu_si128((__m128i*)start);
                sum16 = _mm_add_epi16(sum16, _mm_unpacklo_epi8(in0, _mm_setzero_si128()));
                sum16 = _mm_add_epi16(sum16, _mm_unpackhi_epi8(in0, _mm_setzero_si128()));
                __m128i in1 = _mm_loadu_si128((__m128i*)(start + 16));
                sum16 = _mm_add_epi16(sum16, _mm_unpacklo_epi8(in1, _mm_setzero_si128()));
                sum16 = _mm_add_epi16(sum16, _mm_unpackhi_epi8(in1, _mm_setzero_si128()));
                __m128i in2 = _mm_loadu_si128((__m128i*)(start + 32));
                sum16 = _mm_add_epi16(sum16, _mm_unpacklo_epi8(in2, _mm_setzero_si128()));
                valY += ((uint16_t*)&sum16)[0] + ((uint16_t*)&sum16)[2] + ((uint16_t*)&sum16)[4] +
                        ((uint16_t*)&sum16)[6];
                valU += ((uint16_t*)&sum16)[1] + ((uint16_t*)&sum16)[5];
                valV += ((uint16_t*)&sum16)[3] + ((uint16_t*)&sum16)[7];
            }
            input[0 + x * 3 + y * 3 * inputWidth] = valY * fac - 1.f;
            input[1 + x * 3 + y * 3 * inputWidth] = valU * fac2 - 1.f;
            input[2 + x * 3 + y * 3 * inputWidth] = valV * fac2 - 1.f;
        }
    }
}
}  // namespace htwk
