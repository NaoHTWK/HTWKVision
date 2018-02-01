#include "htwkpngimagesaverlibpng.h"

#include <png.h>

namespace htwk {
namespace image {

PngImageSaverLibPng::PngImageSaverLibPng()
{

}

bool PngImageSaverLibPng::saveRgbaToPngImage(const std::string& filename, uint32_t width, uint32_t height, const std::vector<uint8_t>& rgba)
{
    FILE *fp = fopen(filename.c_str(), "wb");
    if(fp == nullptr) {
        fclose(fp);
        return false;
    }

    png_structp pngPtr = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    if(pngPtr == nullptr) {
        fclose(fp);
        return false;
    }

    png_infop infoPtr = png_create_info_struct(pngPtr);
    if(infoPtr == nullptr) {
        fclose(fp);
        return false;
    }

    if(setjmp(png_jmpbuf(pngPtr))) {
        fclose(fp);
        return false;
    }

//    png_set_compression_strategy(pngPtr, Z_HUFFMAN_ONLY);
    png_set_compression_level(pngPtr, 1);
    //png_set_filter_heuristics(pngPtr, PNG_FILTER_NONE, 0,0,0);
    png_set_filter(pngPtr, PNG_FILTER_TYPE_BASE, PNG_FILTER_NONE);

    png_init_io(pngPtr, fp);

    // Output is 8bit depth, RGBA format.
    png_set_IHDR(
                pngPtr,
                infoPtr,
                width, height,
                8,
                PNG_COLOR_TYPE_RGBA,
                PNG_INTERLACE_NONE,
                PNG_COMPRESSION_TYPE_DEFAULT,
                PNG_FILTER_TYPE_DEFAULT
                );
    png_write_info(pngPtr, infoPtr);

    // To remove the alpha channel for PNG_COLOR_TYPE_RGB format,
    // Use png_set_filler().
    //png_set_filler(png, 0, PNG_FILLER_AFTER);

    uint8_t** rowPointers = new uint8_t*[sizeof(uint8_t) * height];
    for(png_uint_32 row = 0; row < height; row++) {
        rowPointers[row] = (uint8_t*)(rgba.data() + (row*width*4));
    }

    png_write_image(pngPtr, rowPointers);
    png_write_end(pngPtr, nullptr);

    delete [] rowPointers;

    fclose(fp);
    png_destroy_write_struct(&pngPtr, &infoPtr);

    return true;
}


} // image
} // htwk
