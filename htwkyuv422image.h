#ifndef YUV422IMAGE_H
#define YUV422IMAGE_H

#include <cstdint>

#include <string>
#include <vector>

#include <htwkpngimagesaver.h>

namespace htwk {
namespace image {

struct RGB {
    RGB(uint8_t r, uint8_t g, uint8_t b) {
        y  = (uint8_t) ( 0.299f * r + 0.587f * g + 0.114f  * b);
        cb = (uint8_t) (-0.169f * r - 0.331f * g + 0.499f  * b + 128);
        cr = (uint8_t) ( 0.498f * r - 0.419f * g - 0.0813f * b + 128);
    }

    uint8_t y, cb, cr;
};

class Yuv422Image
{
public:
    const int width;
    const int height;

    Yuv422Image(uint8_t* data, int width, int height);
    Yuv422Image(const Yuv422Image& img);
    Yuv422Image(Yuv422Image&&) = delete;
    ~Yuv422Image();
    Yuv422Image& operator=(const Yuv422Image&) = delete;
    Yuv422Image& operator=(Yuv422Image&&) = delete;

    inline void setY(const int x, const int y, const uint8_t value) {
        data[(x+y*width)<<1]=value;
    }

    void setRGB(int x, int y, const RGB rgb) {
        data[(x + y * width) << 1] = rgb.y;
        data[(((x + y * width) >> 1) << 2) + 1] = rgb.cb;
        data[((x + y * width) << 1) | 3] = rgb.cr;
    }

    void drawCircle(int centerX, int centerY, int radius, int yValue = 255u);
    void drawPlus(int x, int y, int r, int yValue = 255u);
    void drawCross(int x, int y, int r, int yValue = 255u);
    void drawRect(int x1, int y1, int x2, int y2, int yValue = 255u);
    void drawRect(int x1, int y1, int x2, int y2, const RGB& rgb);
    void drawLine(int x1, int y1, int x2, int y2, int yValue = 255u);

    enum Filter {
        NONE,
        CONVERT_TO_GREY
    };

    void saveAsPng(const PngImageSaverPtr& pngSaver, const std::string& filename, Filter filter = NONE);

private:
    uint8_t* data;
    const bool isAllocated;
};

} // image
} // htwk

#endif // YUV422IMAGE_H
