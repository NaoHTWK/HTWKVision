#ifndef HTWKCOLORCONVERSION_H
#define HTWKCOLORCONVERSION_H

#include <cstdint>

#include <vector>

namespace htwk {
namespace image  {

/**
 * @brief The ColorConversion class provides util functions to convert from RGBA to YUV422 and back.
 */
class ColorConversion
{
private:
    static uint8_t clip(float d) {
        if(d<0)d=0u;
        if(d>255)d=255u;
        return (uint8_t)d;
    }

public:
    static void rgbaToYuv422(uint8_t *out, const std::vector<uint8_t> &in, uint32_t width, uint32_t height);
    static void argbToYuv422(uint8_t *out, uint8_t *data, uint32_t width, uint32_t height);

    static void yuv422ToRgba(std::vector<uint8_t> &out, const uint8_t *in, uint32_t width, uint32_t height);
    static void yuv422ToRgba(std::vector<uint8_t> &out, std::vector<uint8_t> &in, uint32_t width, uint32_t height);
    static void yuv422ToGrey(std::vector<uint8_t> &out, const uint8_t *in, uint32_t width, const uint32_t height);

    static uint32_t yuv422ToRgba(int32_t y, int32_t cr, int32_t cb) {
        uint8_t r = clip(y+1.402f*(cr-128)+2);
        uint8_t g = clip(y-0.344f*(cb-128)-0.714f*(cr-128));
        uint8_t b = clip(y+1.772f*(cb-128)+2);

        return (0xffu << 24u) | (b<<16u) | (g<<8u) | r;
    }
};

} // image
} // htwk

#endif // HTWKCOLORCONVERSION_H
