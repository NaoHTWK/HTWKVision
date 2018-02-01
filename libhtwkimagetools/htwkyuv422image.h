#ifndef YUV422IMAGE_H
#define YUV422IMAGE_H

#include <cstdint>

#include <string>
#include <vector>

#include <htwkpngimagesaver.h>

namespace htwk {
namespace image {

class Yuv422Image
{
public:
    const int width;
    const int height;

    Yuv422Image(uint8_t* data, const int width, const int height);
    Yuv422Image(const Yuv422Image& img);
    ~Yuv422Image();

    inline void setY(const int x, const int y, const uint8_t value)
    {
        data[(x+y*width)<<1]=value;
    }

    void drawCircle(const int centerX, const int centerY, const int radius, const int yValue = 255u);
    void drawPlus(const int x, const int y, const int r, const int yValue = 255u);
    void drawCross(const int x, const int y, const int r, const int yValue = 255u);
    void drawRect(int x1, int y1, int x2, int y2, const int yValue = 255u);
    void drawLine(int x1, int y1, int x2, int y2, const int yValue = 255u);

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
