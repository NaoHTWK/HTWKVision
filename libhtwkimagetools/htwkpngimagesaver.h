#ifndef HTWKPNGIMAGESAVER_H
#define HTWKPNGIMAGESAVER_H

#include <cstdint>

#include <memory>
#include <string>
#include <vector>

namespace htwk {
namespace image  {

class PngImageSaver
{
public:
    PngImageSaver();

    virtual bool saveRgbaToPngImage(const std::string& filename, uint32_t width, uint32_t height, const std::vector<uint8_t>& rgba) = 0;
};

using PngImageSaverPtr = std::shared_ptr<PngImageSaver>;

PngImageSaverPtr getPngImageSaverInstace();

} // image
} // htwk

#endif // HTWKPNGIMAGESAVER_H
