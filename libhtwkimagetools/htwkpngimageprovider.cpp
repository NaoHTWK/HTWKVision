#include "htwkpngimageprovider.h"

#include "internal/htwkpngimageproviderlodepng.h"
#include "internal/htwkpngimageproviderlibpng.h"

namespace htwk {
namespace image  {

PngImageProvider::PngImageProvider(const uint32_t expectedWidth, const uint32_t expectedHeight)
    : expectedWidth(expectedWidth)
    , expectedHeight(expectedHeight)
{
}

PngImageProvider::~PngImageProvider()
{

}

PngImageProviderPtr getPngImageProviderInstace(const uint32_t width, const uint32_t height)
{
#ifdef LIBPNG_NOT_AVAILABLE
    return std::make_shared<PngImageProviderLodePng>(width, height);
#else
    return std::make_shared<PngImageProviderLibPng>(width, height);
#endif
}
std::string PngImageProvider::createCachedFileName(const std::string& filename)
{
    return filename + ".yuv422";
}

bool PngImageProvider::tryToLoadCached(const std::string& filename, uint8_t* buffer, const size_t bufferSize, float& pitch, float& roll)
{
    std::string cachedFilename = createCachedFileName(filename);
    FILE* fp = fopen(cachedFilename.c_str(), "r");

    if(fp == nullptr)
        return false;

    if(fread(&pitch, sizeof(pitch), 1, fp) != 1) {
        fclose(fp);
        return false;
    }

    if(fread(&roll, sizeof(roll), 1, fp) != 1) {
        fclose(fp);
        return false;
    }

    if(fread(buffer, bufferSize, 1, fp) != 1) {
        fclose(fp);
        return false;
    }

    fclose(fp);
    return true;
}

void PngImageProvider::saveCachedImage(const std::string& filename, const uint8_t* buffer, const size_t bufferSize, const float& pitch, const float& roll)
{
    std::string cachedFilename = createCachedFileName(filename);
    FILE* fp = fopen(cachedFilename.c_str(), "w");

    if(fp == nullptr)
        return;

    if(fwrite(&pitch, sizeof(pitch), 1, fp) != 1)
    {
        printf("Error writing cached file: %s\n", cachedFilename.c_str());
        fclose(fp);
        return;
    }

    if(fwrite(&roll, sizeof(roll), 1, fp) != 1)
    {
        printf("Error writing cached file: %s\n", cachedFilename.c_str());
        fclose(fp);
        return;
    }

    if(fwrite(buffer, bufferSize, 1, fp) != 1)
    {
        printf("Error writing cached file: %s\n", cachedFilename.c_str());
        fclose(fp);
        return;
    }

    fflush(fp);
    fclose(fp);
}


} // images
} // htwk
