#ifndef HTWKPNGIMAGEPROVIDER_LODEPNG_H
#define HTWKPNGIMAGEPROVIDER_LODEPNG_H

#include <cstdint>

#include <vector>

#include "htwkcolorconversion.h"
#include "htwkpngimageprovider.h"

namespace htwk {
namespace image  {

class PngImageProviderLodePng : public PngImageProvider {
private:
    std::vector<uint8_t> imageRGBA;
    std::vector<uint8_t> fileContents;

    htwk::image::ColorConversion colorConversion;

    void loadFile(const std::string& filename, size_t& fileSize);

public:
    PngImageProviderLodePng(const uint32_t expectedWidth, const uint32_t expectedHeight);
    ~PngImageProviderLodePng() override;

    bool loadAsYuv422(const std::string& filename, uint8_t* buffer, const size_t bufferSize, float& pitch, float& roll) override;
};

} // image
} // htwk
#endif //HTWKPNGIMAGEPROVIDER_LODEPNG_H
