#include "field_border_detector.h"

#include <algorithm_ext.h>
#include <emmintrin.h>

namespace htwk {

FieldBorderDetector::FieldBorderDetector(int8_t* lutCb, int8_t* lutCr, HtwkVisionConfig &config)
    : BaseDetector(lutCb, lutCr, config),
      inputWidth(config.fieldBorderWidth),
      inputHeight(config.fieldBorderHeight),
      shouldWeClassify(config.fieldBorderClassifyData),
      fieldBorderFull(config.width, 0),
      imgPreprocessor(lutCb, lutCr, config, config.fieldBorderWidth, config.fieldBorderHeight) {

    if (shouldWeClassify) {
        tflite.loadModelFromFile(config.tflitePath + "/uc-field-border-classifier.tflite",
                                 {1, inputHeight, inputWidth, channels});
        input = tflite.getInputTensor();
    } else {
        input = (float*)aligned_alloc(16, (((channels * inputWidth * inputHeight * sizeof(float)) + 15) / 16) * 16);
    }
}

FieldBorderDetector::~FieldBorderDetector() {
    if (!shouldWeClassify)
        free(input);
}

void FieldBorderDetector::proceed(uint8_t* img) {
    Timer t("FieldBorderDetector", 50);
    EASY_FUNCTION();

    // In the lower cam the field border is always at 0.
    if (!config.isUpperCam)
        return;

//    createInputData(img);
    imgPreprocessor.proceed(img);
    memcpy(input, imgPreprocessor.getScaledImage().data(), sizeof(*input)*channels * inputWidth * inputHeight);

    if (!shouldWeClassify)
        return;

    EASY_BLOCK("TFL FieldBorder");
    tflite.execute();
    EASY_END_BLOCK;

    outputToFieldBorder();
}

void FieldBorderDetector::createInputData(uint8_t* img) {
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
                valY += ((uint16_t*)&sum16)[0] + ((uint16_t*)&sum16)[2] + ((uint16_t*)&sum16)[4] +
                        ((uint16_t*)&sum16)[6];
                valU += ((uint16_t*)&sum16)[1] + ((uint16_t*)&sum16)[5];
                valV += ((uint16_t*)&sum16)[3] + ((uint16_t*)&sum16)[7];
            }

            const float yVal = valY * fac;
            input[0 + x * 3 + y * 3 * inputWidth] = yVal;  // - 1.f;
            input[1 + x * 3 + y * 3 * inputWidth] = valU * fac2 - 1.f;
            input[2 + x * 3 + y * 3 * inputWidth] = valV * fac2 - 1.f;
        }
    }
}

void FieldBorderDetector::outputToFieldBorder() {
    const float* features = tflite.getOutputTensor();
    int numBins = config.fieldBorderWidth;
    int stepWidth = config.width / numBins;
    for (int x = 0; x < config.width; x++) {
        int x1 = (x - stepWidth / 2) / stepWidth;
        if (x1 < 0) {
            x1 = 0;
        }
        if (x1 >= numBins - 2) {
            x1 = numBins - 2;
        }
        int y1 = (int)(features[x1] * config.height);
        int x2 = x1 + 1;
        int y2 = (int)(features[x2] * config.height);
        double f = (x - stepWidth / 2 - x1 * stepWidth * 1.) / stepWidth;
        int y = (int)(y2 * f + y1 * (1 - f));
        if (y < 0)
            y = 0;
        if (y >= config.height) {
            y = config.height - 1;
        }
        fieldBorderFull[x] = y;
    }
}

void FieldBorderDetector::drawInputParameter(uint8_t* yuvImage) {
    const int blockSizeWidth = width / inputWidth;
    const int blockSizeHeight = height / inputHeight;

    for (int ySample = 0; ySample < inputHeight; ySample++) {
        const int startY = ySample * blockSizeHeight;

        for (int xSample = 0; xSample < inputWidth; xSample++) {
            const int startX = xSample * blockSizeWidth;

            const int vcy = (int)((input[0 + channels * xSample + ySample * channels * inputWidth] + 1.f) * 128.f);
            const int vcb = (int)((input[1 + channels * xSample + ySample * channels * inputWidth] + 1.f) * 128.f);
            const int vcr = (int)((input[2 + channels * xSample + ySample * channels * inputWidth] + 1.f) * 128.f);
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

void FieldBorderDetector::drawFieldBorder(uint8_t* yuvImage) {}

}  // namespace htwk
