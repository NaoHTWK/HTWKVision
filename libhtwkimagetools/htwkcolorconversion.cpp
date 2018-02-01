#include "htwkcolorconversion.h"

namespace htwk {
namespace image  {

ColorConversion::ColorConversion()
{

}

void ColorConversion::rgbaToYuv422(uint8_t *out, const std::vector<uint8_t> &in, const uint32_t width, const uint32_t height) const
{
    const uint8_t* data = in.data();

    for (uint32_t y = 0; y < height; y++) {
        for (uint32_t x = 0; x < width; x+=2) {
            const int start = x * 4 + y * width * 4;
            const float r1 = data[start];
            const float g1 = data[start+1];
            const float b1 = data[start+2];

            const float r2 = data[start+4];
            const float g2 = data[start+5];
            const float b2 = data[start+6];

            const int outIdx = x * 2 + y * width * 2;

            /* Y1  */ out[outIdx]   = (uint8_t) ( 0.299f * r1 + 0.587f * g1 + 0.114f  * b1);
            /* Cb1 */ out[outIdx+1] = (uint8_t) (-0.169f * r1 - 0.331f * g1 + 0.499f  * b1 + 128);
            /* Y2  */ out[outIdx+2] = (uint8_t) ( 0.299f * r2 + 0.587f * g2 + 0.114f  * b2);
            /* Cr1 */ out[outIdx+3] = (uint8_t) ( 0.498f * r2 - 0.419f * g2 - 0.0813f * b2 + 128);
        }
    }
}

void ColorConversion::yuv422ToRgba(std::vector<uint8_t> &out, const uint8_t *const in, const uint32_t width, const uint32_t height) const
{
    for(uint32_t py=0;py<height;py++){
        int cbLast=in[(0+py*width)*2+1]&255;
        int crLast=in[(0+py*width)*2+3]&255;
        for(uint32_t px=0;px<width;px++){
            int y=in[(px+py*width)*2]&255;
            if((px&1)==0){
                cbLast=in[(px+py*width)*2+1]&255;
            }else{
                crLast=in[(px+py*width)*2+1]&255;
            }
            int cb=cbLast;
            int cr=crLast;
            out[px * 4 + py * width * 4] = clip(y+1.402f*(cr-128)+2);
            out[1 + px * 4 + py * width * 4] = clip(y-0.344f*(cb-128)-0.714f*(cr-128));
            out[2 + px * 4 + py * width * 4] = clip(y+1.772f*(cb-128)+2);
        }
    }
}

void ColorConversion::yuv422ToRgba(std::vector<uint8_t> &out, std::vector<uint8_t> &in, const uint32_t width, const uint32_t height) const
{
    uint8_t* inArray = &(in[0]);
    yuv422ToRgba(out, inArray, width, height);
}

void ColorConversion::yuv422ToGrey(std::vector<uint8_t> &out, const uint8_t *const in, const uint32_t width, const uint32_t height) const
{
    for(uint32_t py=0;py<height;py++){
        for(uint32_t px=0;px<width;px++){
            int y=in[(px+py*width)*2]&255;

            out[px * 4 + py * width * 4] = clip(y);
            out[1 + px * 4 + py * width * 4] = clip(y);
            out[2 + px * 4 + py * width * 4] = clip(y);
        }
    }
}

} // image
} // htwk
