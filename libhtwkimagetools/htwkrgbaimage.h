#ifndef HTWKRGBAIMAGE_H
#define HTWKRGBAIMAGE_H

#include <stdint.h>

#include <string>
#include <vector>

#include <htwkpngimagesaver.h>

namespace htwk {
namespace image {

class RgbaImage
{
private:
    const uint32_t width;
    const uint32_t height;
    std::vector<uint8_t> mem;

public:
    RgbaImage(uint32_t _width, uint32_t _height) : width(_width), height(_height), mem(4*width*height) {}
    RgbaImage(RgbaImage&&) = default;
    RgbaImage& operator=(const RgbaImage&) = default;

    inline void setPixel(uint32_t x, uint32_t y, uint32_t rgba) {
        *((uint32_t*)(&mem[4*(x+y*width)])) = rgba;
    }

    inline void setPixel(uint32_t x, uint32_t y, uint8_t r, uint8_t g, uint8_t b, uint8_t a) {
        uint32_t rgba = (a << 24) | (r << 16) | (g << 8) | (b << 0);
        setPixel(x, y, rgba);
    }

    inline const std::vector<uint8_t>& getBuffer() const { return mem; }

    void saveAsPng(const PngImageSaverPtr& pngSaver, const std::string& filename);
};

} // image
} // htwk

#endif // HTWKRGBAIMAGE_H
