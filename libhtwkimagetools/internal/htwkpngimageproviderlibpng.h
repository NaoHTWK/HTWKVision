#ifndef HTWKPNGIMAGEPROVIDERLIBPNG_H
#define HTWKPNGIMAGEPROVIDERLIBPNG_H

#include <cstdint>

#include <vector>

#include <htwkcolorconversion.h>
#include <htwkpngimageprovider.h>

namespace htwk {
namespace image {

class PngImageProviderLibPng : public PngImageProvider
{
private:
    std::vector<uint8_t> imageRGBA;
    uint8_t** rowPointers;

    htwk::image::ColorConversion colorConversion;

public:
    PngImageProviderLibPng(const uint32_t expectedWidth, const uint32_t expectedHeight);
    ~PngImageProviderLibPng() override;

    bool loadAsYuv422(const std::string& filename, uint8_t* buffer, const size_t bufferSize, float& pitch, float& roll, float &headPitch, float &headYaw) override;
};

} // images
} // htwk

#endif // HTWKPNGIMAGEPROVIDERLIBPNG_H
