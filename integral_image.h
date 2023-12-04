#ifndef INTEGRAL_IMAGE_H
#define INTEGRAL_IMAGE_H

#include <cstdint>

#include <base_detector.h>

namespace htwk {
class IntegralImage : protected BaseDetector {
private:
    int* integralImg;

public:
    static constexpr int INTEGRAL_SCALE = 2;//only 1,2 or 4
    const int iWidth;
    const int iHeight;

    IntegralImage(int8_t *lutCb, int8_t *lutCr, HtwkVisionConfig& config) __attribute__((nonnull));
    IntegralImage(const IntegralImage&) = delete;
    IntegralImage(IntegralImage&&) = delete;
    IntegralImage& operator=(const IntegralImage&) = delete;
    IntegralImage& operator=(IntegralImage&&) = delete;
    ~IntegralImage();

    inline int getIntegralValue(int x1, int y1, int x2, int y2) const {
        return (integralImg[x2+y2*iWidth]-integralImg[x1+y2*iWidth]-integralImg[x2+y1*iWidth]+integralImg[x1+y1*iWidth]);
    }

    void proceed(uint8_t *img) const __attribute__((nonnull));
    inline const int* getIntegralImg() { return integralImg; }
};
}//namespace htwk
#endif // INTEGRAL_IMAGE_H
