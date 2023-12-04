#include "image_preprocessor.h"

#include <iostream>

#include <easy/profiler.h>
#include <emmintrin.h>

namespace htwk {

htwk::ImagePreprocessor::ImagePreprocessor(int8_t *lutCb, int8_t *lutCr, htwk::HtwkVisionConfig &config, int scaledWidth, int scaledHeight)
    : BaseDetector(lutCb, lutCr, config),
      scaledWidth(scaledWidth),
      scaledHeight(scaledHeight),
      scaledImage(scaledWidth * scaledHeight * 3) {

}

void ImagePreprocessor::proceed(uint8_t *img) {
    Timer t("ImagePreprocessor", 50);
    scaleImage(img);
}

void ImagePreprocessor::scaleImage(uint8_t *img) {
    EASY_FUNCTION();
    const int blockWidth = width / scaledWidth;
    const int blockHeight = height / scaledHeight;

    if (blockWidth == 4) {
        scaleImage4(img);
        return;
    }

    if (blockWidth % 8 != 0)
        std::cerr << __PRETTY_FUNCTION__ << ": Illegal block width!!! " << width << " scaled:" << scaledWidth << std::endl;
    if (blockWidth * blockHeight > 256 * 4)
        std::cerr << __PRETTY_FUNCTION__ << ": Block size too big!!!"  << std::endl;

    float fac = 1.f / (blockWidth * blockHeight * 255);
    float fac2 = 1.f / (blockWidth * blockHeight * 255) * 2;
    for (int y = 0; y < scaledHeight; y++) {
        for (int x = 0; x < scaledWidth; x++) {
            // yy uu yy vv yy uu yy vv
            __m128i sum16 = _mm_setzero_si128();
            const uint8_t* start = img + x * blockWidth * 2 + y * blockHeight * width * 2;
            for (int y2 = 0; y2 < blockHeight; y2++) {
                for (int x2 = 0; x2 < blockWidth / 8; x2++) {
                    __m128i in = _mm_loadu_si128((__m128i*)(start + x2 * 16 + y2 * width * 2));
                    sum16 = _mm_add_epi16(sum16, _mm_unpacklo_epi8(in, _mm_setzero_si128()));
                    sum16 = _mm_add_epi16(sum16, _mm_unpackhi_epi8(in, _mm_setzero_si128()));
                }
            }
            int valY = (int)((uint16_t*)&sum16)[0] + (int)((uint16_t*)&sum16)[2] + (int)((uint16_t*)&sum16)[4] + (int)((uint16_t*)&sum16)[6];
            int valU = (int)((uint16_t*)&sum16)[1] + (int)((uint16_t*)&sum16)[5];
            int valV = (int)((uint16_t*)&sum16)[3] + (int)((uint16_t*)&sum16)[7];
            scaledImage[0 + x * 3 + y * 3 * scaledWidth] = valY * fac;
            scaledImage[1 + x * 3 + y * 3 * scaledWidth] = valU * fac2;
            scaledImage[2 + x * 3 + y * 3 * scaledWidth] = valV * fac2;
        }
    }
}

void ImagePreprocessor::scaleImage4(uint8_t *img) {
    EASY_FUNCTION();
    const int blockWidth = width / scaledWidth;
    const int blockHeight = height / scaledHeight;
    if (blockWidth * blockHeight > 256 * 4)
        std::cerr << __PRETTY_FUNCTION__ << ": Block size too big!!!"  << std::endl;
    float fac = 1.f / (blockWidth * blockHeight * 255);
    float fac2 = 1.f / (blockWidth * blockHeight * 255) * 2;
    for (int y = 0; y < scaledHeight; y++) {
        for (int x = 0; x < scaledWidth; x+=2) {
            // yy uu yy vv yy uu yy vv
            __m128i sum16 = _mm_setzero_si128();
            __m128i sum16_2 = _mm_setzero_si128();
            const uint8_t* start = img + x * blockWidth * 2 + y * blockHeight * width * 2;
            for (int y2 = 0; y2 < blockHeight; y2++) {
                __m128i in = _mm_loadu_si128((__m128i*)(start + y2 * width * 2));
                sum16 = _mm_add_epi16(sum16, _mm_unpacklo_epi8(in, _mm_setzero_si128()));
                sum16_2 = _mm_add_epi16(sum16_2, _mm_unpackhi_epi8(in, _mm_setzero_si128()));
            }
            int valY = (int)((uint16_t*)&sum16)[0] + (int)((uint16_t*)&sum16)[2] + (int)((uint16_t*)&sum16)[4] + (int)((uint16_t*)&sum16)[6];
            int valU = (int)((uint16_t*)&sum16)[1] + (int)((uint16_t*)&sum16)[5];
            int valV = (int)((uint16_t*)&sum16)[3] + (int)((uint16_t*)&sum16)[7];
            scaledImage[0 + x * 3 + y * 3 * scaledWidth] = valY * fac;
            scaledImage[1 + x * 3 + y * 3 * scaledWidth] = valU * fac2;
            scaledImage[2 + x * 3 + y * 3 * scaledWidth] = valV * fac2;
            int valY2 = (int)((uint16_t*)&sum16_2)[0] + (int)((uint16_t*)&sum16_2)[2] + (int)((uint16_t*)&sum16_2)[4] + (int)((uint16_t*)&sum16_2)[6];
            int valU2 = (int)((uint16_t*)&sum16_2)[1] + (int)((uint16_t*)&sum16_2)[5];
            int valV2 = (int)((uint16_t*)&sum16_2)[3] + (int)((uint16_t*)&sum16_2)[7];
            scaledImage[0 + (x+1) * 3 + y * 3 * scaledWidth] = valY2 * fac;
            scaledImage[1 + (x+1) * 3 + y * 3 * scaledWidth] = valU2 * fac2;
            scaledImage[2 + (x+1) * 3 + y * 3 * scaledWidth] = valV2 * fac2;
        }
    }
}

void ImagePreprocessor::createInputDataRaw(const uint8_t* img, std::vector<float>& dest) {
    EASY_FUNCTION();
    const int blockWidth = width / scaledWidth;
    const int blockHeight = height / scaledHeight;
    if (blockWidth % 8 != 0) {
        std::cerr << __FILE__ << __PRETTY_FUNCTION__ << ": Illegal block width!!! " << width << " scaled:" << scaledWidth << std::endl;
        exit(1);
    }
    if (blockWidth * blockHeight > 256 * 4) {
        std::cerr << __FILE__ << __PRETTY_FUNCTION__ << ": Block size too big!!!"  << std::endl;
        exit(1);
    }
    float fac = 1.f / (blockWidth * blockHeight);
    float fac2 = 1.f / (blockWidth * blockHeight) * 2;
    for (int y = 0; y < scaledHeight; y++) {
        for (int x = 0; x < scaledWidth; x++) {
            // yy uu yy vv yy uu yy vv
            __m128i sum16 = _mm_setzero_si128();
            const uint8_t* start = img + x * blockWidth * 2 + y * blockHeight * width * 2;
            for (int y2 = 0; y2 < blockHeight; y2++) {
                for (int x2 = 0; x2 < blockWidth / 8; x2++) {
                    __m128i in = _mm_loadu_si128((__m128i*)(start + x2 * 16 + y2 * width * 2));
                    sum16 = _mm_add_epi16(sum16, _mm_unpacklo_epi8(in, _mm_setzero_si128()));
                    sum16 = _mm_add_epi16(sum16, _mm_unpackhi_epi8(in, _mm_setzero_si128()));
                }
            }
            int valY = (int)((uint16_t*)&sum16)[0] + (int)((uint16_t*)&sum16)[2] + (int)((uint16_t*)&sum16)[4] + (int)((uint16_t*)&sum16)[6];
            int valU = (int)((uint16_t*)&sum16)[1] + (int)((uint16_t*)&sum16)[5];
            int valV = (int)((uint16_t*)&sum16)[3] + (int)((uint16_t*)&sum16)[7];
            dest[0 + x * 3 + y * 3 * scaledWidth] = valY * fac;
            dest[1 + x * 3 + y * 3 * scaledWidth] = valU * fac2;
            dest[2 + x * 3 + y * 3 * scaledWidth] = valV * fac2;
        }
    }
}

void ImagePreprocessor::drawScaledImage(uint8_t *yuvImage) {
    const int blockSizeWidth = width / scaledWidth;
    const int blockSizeHeight = height / scaledHeight;
    const int channels = 3;

    for (int ySample = 0; ySample < scaledHeight; ySample++) {
        const int startY = ySample * blockSizeHeight;

        for (int xSample = 0; xSample < scaledWidth; xSample++) {
            const int startX = xSample * blockSizeWidth;

            const int vcy = (int)(scaledImage[0 + channels * xSample + ySample * channels * scaledWidth] * 255.f);
            const int vcb = (int)(scaledImage[1 + channels * xSample + ySample * channels * scaledWidth] * 255.f);
            const int vcr = (int)(scaledImage[2 + channels * xSample + ySample * channels * scaledWidth] * 255.f);
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
