#ifndef HTWKPNGIMAGESAVERLIBPNG_H
#define HTWKPNGIMAGESAVERLIBPNG_H

#include <htwkpngimagesaver.h>

namespace htwk {
namespace image {

class PngImageSaverLibPng : public PngImageSaver
{
public:
    PngImageSaverLibPng();

    bool saveRgbaToPngImage(const std::string& filename, uint32_t width, uint32_t height, const std::vector<uint8_t>& rgba) override;
};

} // image
} // htwk

#endif // HTWKPNGIMAGESAVERLIBPNG_H
