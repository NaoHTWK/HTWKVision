#include "htwkpngimageproviderlibpng.h"

#include <png.h>

namespace htwk {
namespace image {

PngImageProviderLibPng::~PngImageProviderLibPng()
{
    delete [] rowPointers;
}

PngImageProviderLibPng::PngImageProviderLibPng(const uint32_t expectedWidth, const uint32_t expectedHeight)
    : PngImageProvider(expectedWidth, expectedHeight)
{
    imageRGBA.reserve(expectedWidth*expectedHeight*4 /* RGBA */);

    rowPointers = new uint8_t*[sizeof(uint8_t) * expectedHeight];
    for(png_uint_32 row = 0; row < expectedHeight; row++) {
        rowPointers[row] = imageRGBA.data() + (row*expectedWidth*4);
    }
}

bool PngImageProviderLibPng::loadAsYuv422(const std::string& filename, uint8_t* buffer, const size_t bufferSize, float& pitch, float& roll, float& headPitch, float& headYaw)
{
    if(tryToLoadCached(filename, buffer, bufferSize, pitch, roll, headPitch, headYaw))
        return true;

    int rc = 0;
    FILE* fp = fopen(filename.c_str(), "rb");

    if(fp == nullptr) {
        fprintf(stderr, "PngImageProviderLibPng::loadAsYuv422: fopen of %s failed\n", filename.c_str());
        return false;
    }

    png_structp pngPtr = png_create_read_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);

    if(pngPtr == nullptr) {
        fprintf(stderr, "PngImageProviderLibPng::loadAsYuv422: png_create_read_struct failed\n");
        fclose(fp);
        return false;
    }

    png_infop infoPtr = png_create_info_struct(pngPtr);
    if(infoPtr == nullptr) {
        fprintf(stderr, "PngImageProviderLibPng::loadAsYuv422: png_create_info_struct failed\n");
        fclose(fp);
        return false;
    }

    if(setjmp(png_jmpbuf(pngPtr))) {
        fprintf(stderr, "PngImageProviderLibPng::loadAsYuv422: png_jmpbuf failed\n");
        fclose(fp);
        return false;
    }

    png_init_io(pngPtr, fp);

    png_read_info(pngPtr, infoPtr);

    png_uint_32 width, height;
    int bitDepth, colorType, interlaceType;

    rc = png_get_IHDR(pngPtr, infoPtr, &width, &height, &bitDepth, &colorType, &interlaceType, nullptr, nullptr);
    if(rc == 0)  {
        fprintf(stderr, "PngImageProviderLibPng::loadAsYuv422: png_get_IHDR failed\n");
        png_destroy_read_struct(&pngPtr, &infoPtr, nullptr);
        fclose(fp);
        return false;
    }

    png_textp textPtr;
    int numText;

    if(png_get_text(pngPtr, infoPtr, &textPtr, &numText) <= 0)
    {
        fprintf(stderr, "PngImageProviderLibPng::loadAsYuv422: png does not include any text!\n");
        png_destroy_read_struct(&pngPtr, &infoPtr, nullptr);
        fclose(fp);
        return false;
    }

    for(int i = 0; i < numText; i++) {
        std::string key(textPtr[i].key);
        std::string value(textPtr[i].text);

        if(textPtr[i].compression == PNG_TEXT_COMPRESSION_zTXt) {
            fprintf(stderr, "PngImageProviderLibPng::loadAsYuv422: skip compressed text %s!\n", key.c_str());
            continue;
        }

        if(key == "pitch") {
            pitch = std::stof(value);
        } else if(key == "roll") {
            roll = std::stof(value);
        } else if(key == "headPitch") {
            headPitch = std::stof(value);
        } else if(key == "headYaw") {
            headYaw = std::stof(value);
        }
    }


    if(width != expectedWidth || height != expectedHeight) {
        fprintf(stderr, "PngImageProviderLibPng::loadAsYuv422: png_get_IHDR failed\n");
        png_destroy_read_struct(&pngPtr, &infoPtr, nullptr);
        fclose(fp);
        return false;
    }

    if(bitDepth == 16) {
        png_set_strip_16(pngPtr);
    }

    if(bitDepth < 8) {
        png_set_packing( pngPtr );
    }

    if(colorType & PNG_COLOR_TYPE_PALETTE) {
        png_set_expand( pngPtr );
    }

    if(colorType == PNG_COLOR_TYPE_GRAY && bitDepth < 8) {
        png_set_expand_gray_1_2_4_to_8(pngPtr);
    }

    if(png_get_valid(pngPtr, infoPtr, PNG_INFO_tRNS)) {
        png_set_tRNS_to_alpha( pngPtr );
    }

    if(!static_cast<bool>(colorType & PNG_COLOR_MASK_ALPHA)) {
        png_set_filler(pngPtr, 0xff, PNG_FILLER_AFTER);
    }

    png_read_update_info(pngPtr, infoPtr);

    png_read_image(pngPtr, rowPointers);
    png_read_end(pngPtr, infoPtr);
    png_destroy_read_struct(&pngPtr, &infoPtr, nullptr);

    fclose(fp);

    colorConversion.rgbaToYuv422(buffer, imageRGBA, expectedWidth, expectedHeight);

    if(cacheLoadedImages)
        saveCachedImage(filename, buffer, bufferSize, pitch, roll, headPitch, headYaw);

    return true;
}

} // imagesb
} // htwk
