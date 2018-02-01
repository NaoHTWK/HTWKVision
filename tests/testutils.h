#ifndef TESTUTILS_H
#define TESTUTILS_H

#include <algorithm>
#include <deque>
#include <iostream>
#include <stdint.h>
#include <string>
#include <vector>

#include <lodepng/lodepng.h>

#define STD_WIDTH  640
#define STD_HEIGHT 480

class TestUtils
{
public:
    TestUtils();

    static uint8_t* loadFile(std::string& filename, uint32_t& width, uint32_t& height);
    static void rgbaToYuv422(uint8_t *out, const std::vector<uint8_t> &in, int width, int height);

    static std::string testImagePath;
};

#endif // TESTUTILS_H
