#include "htwkyuv422image.h"

#include <cmath>
#include <cstring>

#include <htwkcolorconversion.h>

namespace htwk {
namespace image {

Yuv422Image::Yuv422Image(uint8_t* data, const int width, const int height)
    : width(width)
    , height(height)
    , data(data)
    , isAllocated(false)
{
}

Yuv422Image::Yuv422Image(const Yuv422Image& img)
    : width(img.width)
    , height(img.height)
    , isAllocated(true)
{
    data = new uint8_t[2*width*height];
    memcpy(data, img.data, 2*width*height);
}

Yuv422Image::~Yuv422Image()
{
    if(isAllocated)
        delete [] data;
}


void Yuv422Image::drawCircle(const int centerX, const int centerY, const int radius, const int yValue)
{
    for(float a=0;a<M_PI*2;a+=0.01){
        int nx=centerX+std::sin(a)*radius;
        int ny=centerY+std::cos(a)*radius;

        if(nx<0||ny<0||nx>=width||ny>=height)
            continue;

        setY(nx, ny, yValue);
    }
}

void Yuv422Image::drawPlus(const int x, const int y, const int r, const int yValue)
{
    for(int d=-r;d<=r;d++){
        int px1=x+d;
        int py1=y;
        if(px1<0||px1>=width)continue;
        if(py1<0||py1>=height)continue;
        setY(px1, py1, yValue);
    }

    for(int d=-r;d<=r;d++){
        int px2=x;
        int py2=y+d;
        if(px2<0||px2>=width)continue;
        if(py2<0||py2>=height)continue;
        setY(px2, py2, yValue);
    }
}

void Yuv422Image::drawCross(const int x, const int y, const int r, const int yValue)
{
    for(int d=-r;d<=r;d++){
        int px1=x+d;
        int px2=x-d;
        int py1=y+d;
        if(px1<0||px1>=width)continue;
        if(px2<0||px2>=width)continue;
        if(py1<0||py1>=height)continue;
        setY(px1, py1, yValue);
        setY(px2, py1, yValue);
    }
}

void Yuv422Image::drawRect(int x1, int y1, int x2, int y2, const int yValue) {
    if(y1 > y2) {
        int tmp;
        tmp = y1;
        y1 = y2;
        y2 = tmp;
    }

    if(x1 > x2) {
        int tmp;
        tmp = x1;
        x1 = x2;
        x2 = tmp;
    }

    if(x1<0)x1=0;
    if(x1>=width)x1=width-1;
    if(x2<0)x2=0;
    if(x2>=width)x2=width-1;
    if(y1<0)y1=0;
    if(y1>=height)y1=height-1;
    if(y2<0)y2=0;
    if(y2>=height)y2=height-1;
    for(int x=x1;x<=x2;x++){
        setY(x, y1, yValue);
        setY(x, y2, yValue);
    }
    for(int y=y1;y<=y2;y++){
        setY(x1, y, yValue);
        setY(x2, y, yValue);
    }
}

void Yuv422Image::drawRect(int x1, int y1, int x2, int y2, const RGB &rgb) {
    if(x1<0)x1=0;
    if(x1>=width)x1=width-1;
    if(x2<0)x2=0;
    if(x2>=width)x2=width-1;
    if(y1<0)y1=0;
    if(y1>=height)y1=height-1;
    if(y2<0)y2=0;
    if(y2>=height)y2=height-1;
    for(int x=x1;x<=x2;x++){
        setRGB(x, y1, rgb);
        setRGB(x, y2, rgb);
    }
    for(int y=y1;y<=y2;y++){
        setRGB(x1, y, rgb);
        setRGB(x2, y, rgb);
    }
}

void Yuv422Image::drawLine(int x1, int y1, int x2, int y2, const int yValue) {
    float dx = x2-x1;
    float dy = y2-y1;
    if (dy != 0) {
        for (int y = 0; y < height; y++) {
            float n = (y-y1)/dy;
            if (n < 0 || n > 1) continue;
            int x = (int)(x1+n*dx);
            if (x < 0 || x >= width) continue;
            setY(x, y, yValue);
        }
    }
    if (dx != 0) {
        for (int x = 0; x < width; x++) {
            float n = (x-x1)/dx;
            if (n < 0 || n > 1) continue;
            int y = (int)(y1+n*dy);
            if (y < 0 || y >= height) continue;
            setY(x, y, yValue);
        }
    }
}

void Yuv422Image::saveAsPng(const PngImageSaverPtr& pngSaver, const std::string& filename, Filter filter)
{
    ColorConversion conv;
    std::vector<uint8_t> outRGBA(width * height * 4, 255);
    if (filter == NONE){
        conv.yuv422ToRgba(outRGBA, data, width, height);
    }else if(filter == CONVERT_TO_GREY){
        conv.yuv422ToGrey(outRGBA, data, width, height);
    }
    pngSaver->saveRgbaToPngImage(filename, width, height, outRGBA);
}

} // image
} // htwk
