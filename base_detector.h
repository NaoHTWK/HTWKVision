#ifndef __BASE_DETECTOR_H__
#define __BASE_DETECTOR_H__

#include <utility>

#include <cstdint>

#include "color.h"
#include "range_check.h"

namespace htwk {
class BaseDetector {
protected:
    const int width;
    const int height;
    const int8_t *lutCb;
    const int8_t *lutCr;

public:
    inline uint8_t getY(const uint8_t * const img, int32_t x, int32_t y) const __attribute__((nonnull)) __attribute__((pure)) {
        CHECK_RANGE(x,0,width-1);
        CHECK_RANGE(y,0,height-1);
        return img[(x + y * width) << 1];
    }

    inline uint8_t getCb(const uint8_t * const img, int32_t x, int32_t y) const __attribute__((nonnull)) __attribute__((pure)) {
        CHECK_RANGE(x,0,width-1);
        CHECK_RANGE(y,0,height-1);
        return img[(((x + y * width) >> 1) << 2) + 1];
    }

    inline uint8_t getCr(const uint8_t * const img, int32_t x, int32_t y) const __attribute__((nonnull)) __attribute__((pure)) {
        CHECK_RANGE(x,0,width-1);
        CHECK_RANGE(y,0,height-1);
        return img[((x + y * width) << 1) | 3];
    }

    inline void setY(uint8_t* const img, const int32_t x,int32_t y, const uint8_t c) __attribute__((nonnull)){
        CHECK_RANGE(x,0,width-1);
        CHECK_RANGE(y,0,height-1);
        img[(x+y*width)<<1]=c;
    }

    inline color getYCbCr(const uint8_t * const img, int32_t x, int32_t y) const __attribute__((nonnull)) __attribute__((pure)) {
        CHECK_RANGE(x,0,width-1);
        CHECK_RANGE(y,0,height-1);

        return color((uint8_t)(img[(x + y * width) << 1]), (uint8_t)(img[(((x + y * width) >> 1) << 2) + 1]), (uint8_t)(img[((x + y * width) << 1) | 3]));
    }


    BaseDetector(const int _width, const int _height, const int8_t* _lutCb, const int8_t* _lutCr) :
        width(_width), height(_height), lutCb(_lutCb), lutCr(_lutCr) {}
};

}

#endif
