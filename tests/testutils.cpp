#include "testutils.h"

std::string TestUtils::testImagePath;

TestUtils::TestUtils()
{

}

uint8_t* TestUtils::loadFile(std::string& filename, uint32_t& width, uint32_t& height)
{
    std::vector<uint8_t> imageRGBA;
    uint32_t error = lodepng::decode(imageRGBA, width, height, filename);

    //if there's an error, display it
    if(error) {
        std::cout << "decoder error " << error << ": " << lodepng_error_text(error) << std::endl;
        exit(1);
    }

    uint8_t *imageYUV422 = NULL;
    if(posix_memalign((void**)&imageYUV422, 16, sizeof(uint8_t) * 2 * width * height) != 0) {
        std::cout << "error allocating aligned memory! reason: " << strerror(errno) << std::endl;
        exit(1);
    }


    if(imageYUV422 == NULL) {
        std::cout << "Couldn't allocate yuv memory. Exit now." << std::endl;
        exit(1);
    }

    if(width == STD_WIDTH && height == STD_HEIGHT)
        rgbaToYuv422(imageYUV422, imageRGBA, width, height);

    return imageYUV422;
}

void TestUtils::rgbaToYuv422(uint8_t *out, const std::vector<uint8_t> &in, int width, int height)
{
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int r = in[x * 4 + y * width * 4];
            int g = in[1 + x * 4 + y * width * 4];
            int b = in[2 + x * 4 + y * width * 4];
            out[x * 2 + y * width * 2] = (uint8_t) (0.299 * r + 0.587 * g + 0.114 * b);
            if (x % 2 == 0) {
                out[1 + x * 2 + y * width * 2] =
                        (uint8_t) (-0.169 * r - 0.331 * g + 0.499 * b + 128);
            } else {
                out[1 + x * 2 + y * width * 2] =
                        (uint8_t) (0.498 * r - 0.419 * g - 0.0813 * b + 128);
            }
        }
    }
}
