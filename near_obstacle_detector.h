#ifndef __NEAROBSTACLEDETECT_H__
#define __NEAROBSTACLEDETECT_H__

#include <cstdint>
#include <exponential_moving_average.h>

#include "base_detector.h"

namespace htwk {

class FieldColorDetector;

class NearObstacleDetector : protected BaseDetector
{
private:

    static const int pixelSpacing;
    static constexpr float smoothing = 0.9f;
    static const float threshold;
    static const float gain;

    EMA obstacleL{0, smoothing};
    EMA obstacleC{0, smoothing};
    EMA obstacleR{0, smoothing};

    float getProb(float s);

public:
    NearObstacleDetector(int _width, int _height, int8_t *_lutCb, int8_t *_lutCr) __attribute__((nonnull)) : BaseDetector(_width, _height, _lutCb, _lutCr) {}
    ~NearObstacleDetector() {}

    void proceed(const uint8_t * const img, FieldColorDetector * const field) __attribute__((nonnull));

    float getObstacleLeft();
    float getObstacleCenter();
    float getObstacleRight();
};

}
#endif
