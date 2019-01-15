#include "htwkpngimagesaverlodepng.h"

#include <lodepng.h>

namespace htwk {
namespace image  {

PngImageSaverLodePng::PngImageSaverLodePng()
{
}

PngImageSaverLodePng::~PngImageSaverLodePng()
{
}

bool PngImageSaverLodePng::saveRgbaToPngImage(const std::string& filename, uint32_t width, uint32_t height, const std::vector<uint8_t>& rgba)
{
    lodepng::encode(filename, rgba, width, height);
    return true;
}

} // image
} // htwk
