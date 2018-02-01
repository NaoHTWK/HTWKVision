#ifndef HTWKPNGIMAGESAVERLODEPNG_H
#define HTWKPNGIMAGESAVERLODEPNG_H

#include <htwkpngimagesaver.h>

namespace htwk {
namespace image  {

class PngImageSaverLodePng : public PngImageSaver
{
public:
    PngImageSaverLodePng();

    bool saveRgbaToPngImage(const std::string& filename, uint32_t width, uint32_t height, const std::vector<uint8_t>& rgba) override;
};

} // image
} // htwk

#endif // HTWKPNGIMAGESAVERLODEPNG_H
