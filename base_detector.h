#pragma once

#include <color.h>
#include <htwk_vision_config.h>
#include <range_check.h>

#include <chrono>
#include <cstdint>
#include <utility>

namespace htwk {

class Timer {
public:
    Timer(std::string name, long timeout_ms) : name(std::move(name)), timeout_ms(timeout_ms) {}
    ~Timer() {
        std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
        if (now > start + std::chrono::milliseconds(timeout_ms))
            printf("%s took %ldms!\n", name.c_str(),
                   std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count());
    }

private:
    std::string name;
    long timeout_ms;
    std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();
};

class BaseDetector {
protected:
    const int8_t* lutCb;
    const int8_t* lutCr;
    HtwkVisionConfig& config;
    const int width;
    const int height;

public:
    inline color getColor(const uint8_t* const img, int32_t x, int32_t y) const __attribute__((nonnull))
    __attribute__((pure)) {
        CHECK_RANGE(x, 0, width - 1);
        CHECK_RANGE(y, 0, height - 1);
        return {img[(x + y * width) << 1], img[(((x + y * width) >> 1) << 2) + 1], img[((x + y * width) << 1) | 3]};
    }
    inline uint8_t getY(const uint8_t* const img, int32_t x, int32_t y) const __attribute__((nonnull))
    __attribute__((pure)) {
        CHECK_RANGE(x, 0, width - 1);
        CHECK_RANGE(y, 0, height - 1);
        return img[(x + y * width) << 1];
    }

    inline uint8_t getCb(const uint8_t* const img, int32_t x, int32_t y) const __attribute__((nonnull))
    __attribute__((pure)) {
        CHECK_RANGE(x, 0, width - 1);
        CHECK_RANGE(y, 0, height - 1);
        return img[(((x + y * width) >> 1) << 2) + 1];
    }

    inline uint8_t getCr(const uint8_t* const img, int32_t x, int32_t y) const __attribute__((nonnull))
    __attribute__((pure)) {
        CHECK_RANGE(x, 0, width - 1);
        CHECK_RANGE(y, 0, height - 1);
        return img[((x + y * width) << 1) | 3];
    }

    inline void setY(uint8_t* const img, const int32_t x, int32_t y, const uint8_t c) __attribute__((nonnull)) {
        CHECK_RANGE(x, 0, width - 1);
        CHECK_RANGE(y, 0, height - 1);
        img[(x + y * width) << 1] = c;
    }

    inline void setYCbCr(uint8_t* img, int32_t x, int32_t y, uint8_t yv, uint8_t cb, uint8_t cr) const {
        CHECK_RANGE(x, 0, width - 1);
        CHECK_RANGE(y, 0, height - 1);

        img[(x + y * width) << 1] = yv;
        img[(((x + y * width) >> 1) << 2) + 1] = cb;
        img[((x + y * width) << 1) | 3] = cr;
    }

    inline void setColor(uint8_t* img, int32_t x, int32_t y, const color& c) const {
        CHECK_RANGE(x, 0, width - 1);
        CHECK_RANGE(y, 0, height - 1);

        img[(x + y * width) << 1] = c.cy;
        img[(((x + y * width) >> 1) << 2) + 1] = c.cb;
        img[((x + y * width) << 1) | 3] = c.cr;
    }
    //    inline color getYCbCr(const uint8_t * const img, int32_t x, int32_t y) const __attribute__((nonnull))
    //    __attribute__((pure)) {
    //        CHECK_RANGE(x,0,width-1);
    //        CHECK_RANGE(y,0,height-1);

    //        return color((uint8_t)(img[(x + y * width) << 1]),
    //                     (uint8_t)(img[(((x + y * width) >> 1) << 2) + 1]),
    //                     (uint8_t)(img[((x + y * width) << 1) | 3]));
    //    }

    BaseDetector(const int8_t* lutCb, const int8_t* lutCr, HtwkVisionConfig& config)
        : lutCb(lutCb), lutCr(lutCr), config(config), width(config.width), height(config.height) {}
};

}  // namespace htwk
