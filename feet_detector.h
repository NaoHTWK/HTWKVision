#ifndef __FEET_DETECTOR_H__
#define __FEET_DETECTOR_H__


#include <cstdint>
#include <vector>

#include <field_color_detector.h>
#include <point_2d.h>
#include <color.h>

namespace htwk {

class FeetDetector{
private:
    static const int q=4;
    int *feetBorder;
    int *footPattern;
    int patternWidth;
    int patternHeight;
    double minRating;
    int feetX;
    int feetY;
    bool found;
    float avgX;
    float avgY;
    bool avgAvailable;
    double *feetRating;

    int width, height;
    int *lutCb;
    int *lutCr;

    /*This class souldn't be copied */
    FeetDetector();
    FeetDetector(FeetDetector& h);
    void operator=(FeetDetector const&);

    inline uint8_t getY(const uint8_t * const img,int32_t x,int32_t y) const __attribute__((nonnull)) __attribute__((pure)){
        CHECK_RANGE(x,0,width-1);
        CHECK_RANGE(y,0,height-1);
        return img[(x+y*width)<<1];
    }
    inline uint8_t getCb(const uint8_t * const img,int32_t x,int32_t y) const __attribute__((nonnull)) __attribute__((pure)){
        CHECK_RANGE(x,0,width-1);
        CHECK_RANGE(y,0,height-1);
        return img[((x+y*width)<<1)+lutCb[x]];
    }
    inline uint8_t getCr(const uint8_t * const img,int32_t x,int32_t y) const __attribute__((nonnull)) __attribute__((pure)){
        CHECK_RANGE(x,0,width-1);
        CHECK_RANGE(y,0,height-1);
        return img[((x+y*width)<<1)+lutCr[x]];
    }
    void createFootPattern();

public:
    inline void setY(uint8_t * const img,int32_t x,int32_t y, uint8_t c) const __attribute__((nonnull)){
        CHECK_RANGE(x,0,width-1);
        CHECK_RANGE(y,0,height-1);
        img[(x+y*width)<<1]=c;
    }

    FeetDetector(int _width, int _height, int * _lutCb, int * _lutCr) __attribute__((nonnull));
    ~FeetDetector();
    void proceed(const uint8_t * const img, FieldColorDetector *field, color ball, bool ballFound, bool useDetection) __attribute__((nonnull));
    bool isAvailable() const;
    bool isInterpolated() const;
    point_2d getBase() const;
};

}  // namespace htwk

#endif  // __FEET_DETECTOR_H__
