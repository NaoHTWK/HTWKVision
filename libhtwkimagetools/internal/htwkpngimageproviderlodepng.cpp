#include "htwkpngimageproviderlodepng.h"

#include <fstream>

#include <lodepng.h>

namespace htwk {
namespace image {

PngImageProviderLodePng::PngImageProviderLodePng(const uint32_t expectedWidth, const uint32_t expectedHeight)
    : PngImageProvider(expectedWidth, expectedHeight)
    , imageRGBA(expectedWidth*expectedHeight*4)
    , fileContents(1*1024*1024)
{
}

PngImageProviderLodePng::~PngImageProviderLodePng()
{
}

void PngImageProviderLodePng::loadFile(const std::string& filename, size_t& fileSize)
{
    std::ifstream file(filename.c_str(), std::ios::in|std::ios::binary|std::ios::ate);

    if(file.seekg(0, std::ios::end).good()) {
        fileSize = file.tellg();
    } else {
        fprintf(stderr, "PngImageProviderLodePng: Error seeking to end of: %s\n", filename.c_str());
        exit(1);
    }

    if(file.seekg(0, std::ios::beg).good()) {
        fileSize -= file.tellg();
    } else {
        fprintf(stderr, "PngImageProviderLodePng: Error seeking to start of: %s\n", filename.c_str());
        exit(1);
    }

    /*read contents of the file into the vector*/
    if(fileContents.size() < fileSize) {
        fileContents.resize(fileSize);
    }

    if(fileSize > 0) file.read((char*)(&fileContents[0]), fileSize);
}

bool PngImageProviderLodePng::loadAsYuv422(const std::string& filename, uint8_t* buffer, const size_t bufferSize, float& pitch, float& roll)
{
    if(tryToLoadCached(filename, buffer, bufferSize, pitch, roll))
        return true;

    uint32_t width = 0;
    uint32_t height = 0;
    size_t fileSize;

    fileContents.clear();
    imageRGBA.clear();

    lodepng::State state;
    loadFile(filename, fileSize);
    uint32_t error = lodepng::decode(imageRGBA, width, height, state, fileContents.data(), fileSize);

    if(error) {
        fprintf(stderr, "PngImageProviderLodePng: PNG decoder error %u: %s\n", error, lodepng_error_text(error));
        exit(1);
    }

    if(width != expectedWidth || height != expectedHeight) {
        fprintf(stderr, "PngImageProviderLodePng: Image %s has unexected resolution (%ux%u) instead (%ux%u) skip it.\n", filename.c_str(), width, height, expectedWidth, expectedHeight);
        return false;
    }

    // Searching for metainformation of the image
    for(size_t i = 0; i < state.info_png.text_num; i++) {
        if (std::string(state.info_png.text_keys[i])=="pitch") {
            pitch = std::stof(state.info_png.text_strings[i]);
        } else if (std::string(state.info_png.text_keys[i])=="roll") {
            roll = std::stof(state.info_png.text_strings[i]);
        }
    }

    if(bufferSize < 2*width*height)
        throw std::invalid_argument("PngImageProviderLodePng: buffer size must be 2*width*height");

    colorConversion.rgbaToYuv422(buffer, imageRGBA, width, height);

    if(cacheLoadedImages)
        saveCachedImage(filename, buffer, bufferSize, pitch, roll);

    return true;
}

} // image
} // htwk
