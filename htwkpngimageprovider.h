#ifndef HTWKPNGIMAGEPROVIDER_H
#define HTWKPNGIMAGEPROVIDER_H

#include <cstdint>

#include <memory>
#include <string>

#include <htwkpngmetadata.h>

namespace htwk {
namespace image  {

class PngImageProvider
{
protected:
    const uint32_t expectedWidth;
    const uint32_t expectedHeight;

    bool cacheLoadedImages = false;

    bool tryToLoadCached(const std::string& filename, uint8_t* buffer, size_t bufferSize, PngMetadata &metadata);
    void saveCachedImage(const std::string& filename, const uint8_t* buffer, size_t bufferSize, PngMetadata &metadata);
    std::string createCachedFileName(const std::string& filename);

public:
    PngImageProvider(uint32_t expectedWidth, uint32_t expectedHeight);
    virtual ~PngImageProvider() = default;
    PngImageProvider(const PngImageProvider&) = delete;
    PngImageProvider(PngImageProvider&&) = delete;
    PngImageProvider& operator=(const PngImageProvider&) = delete;
    PngImageProvider& operator=(PngImageProvider&&) = delete;

    virtual bool loadAsYuv422(const std::string& filename, uint8_t* buffer, size_t bufferSize, PngMetadata& metadata) = 0;

    void setCacheLoadedImage(bool enabled) { cacheLoadedImages = enabled; }
};

using PngImageProviderPtr = std::shared_ptr<PngImageProvider>;

PngImageProviderPtr getPngImageProviderInstace(uint32_t width, uint32_t height);

} // image
} // htwk

#endif // HTWKIMAGEPROVIDER_H
