#ifndef INTEGRAL_IMAGE_H
#define INTEGRAL_IMAGE_H

#include <cstdint>

#include <base_detector.h>

namespace htwk {
class IntegralImage : protected BaseDetector {
private:
    int* integralImg;

public:
    static constexpr int SRC_IMAGE_WIDTH = 640;
    static constexpr int SRC_IMAGE_HEIGHT = 480;
    static constexpr int INTEGRAL_SCALE = 2;//only 1,2 or 4
    static constexpr int iWidth  = SRC_IMAGE_WIDTH/INTEGRAL_SCALE;
    static constexpr int iHeight = SRC_IMAGE_HEIGHT/INTEGRAL_SCALE;

    IntegralImage(int width, int height, int8_t *lutCb, int8_t *lutCr) __attribute__((nonnull));
    ~IntegralImage();

    inline int getIntegralValue(int x1, int y1, int x2, int y2) const {
        return (integralImg[x2+y2*iWidth]-integralImg[x1+y2*iWidth]-integralImg[x2+y1*iWidth]+integralImg[x1+y1*iWidth]);
    }

    void proceed(uint8_t *img) const __attribute__((nonnull));
    inline const int* getIntegralImg() { return integralImg; }
};
}//namespace htwk
#endif // INTEGRAL_IMAGE_H
