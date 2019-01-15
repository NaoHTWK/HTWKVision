#ifndef HTWKPNGIMAGEPROVIDER_H
#define HTWKPNGIMAGEPROVIDER_H

#include <cstdint>

#include <memory>
#include <string>

namespace htwk {
namespace image  {

class PngImageProvider
{
protected:
    const uint32_t expectedWidth;
    const uint32_t expectedHeight;

    bool cacheLoadedImages = false;

    bool tryToLoadCached(const std::string& filename, uint8_t* buffer, const size_t bufferSize, float& pitch, float& roll, float &headPitch, float &headYaw);
    void saveCachedImage(const std::string& filename, const uint8_t* buffer, const size_t bufferSize, const float& pitch, const float& roll, const float& headPitch, const float& headYaw);
    std::string createCachedFileName(const std::string& filename);

public:
    PngImageProvider(const uint32_t expectedWidth, const uint32_t expectedHeight);
    virtual ~PngImageProvider();

    virtual bool loadAsYuv422(const std::string& filename, uint8_t* buffer, const size_t bufferSize, float& pitch, float& roll, float &headPitch, float &headYaw) = 0;

    void setCacheLoadedImage(bool enabled) { cacheLoadedImages = enabled; }
};

using PngImageProviderPtr = std::shared_ptr<PngImageProvider>;

PngImageProviderPtr getPngImageProviderInstace(const uint32_t width, const uint32_t height);

} // image
} // htwk

#endif // HTWKIMAGEPROVIDER_H
