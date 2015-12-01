#ifndef __NEAROBSTACLEDETECT_H__
#define __NEAROBSTACLEDETECT_H__

#include <stdint.h>

#include <base_detector.h>

namespace htwk {

class FieldColorDetector;

class NearObstacleDetector : protected BaseDetector
{
private:

    static const int pixelSpacing;
    static const float smoothing;
    static const float threshold;
    static const float gain;

    float obstacleL;
    float obstacleC;
    float obstacleR;

    float getProb(float s);

public:
    NearObstacleDetector(int width, int height, int8_t *lutCb, int8_t *lutCr) __attribute__((nonnull));
    ~NearObstacleDetector();

    void proceed(const uint8_t * const img, FieldColorDetector * const field) __attribute__((nonnull));

    float getObstacleLeft();
    float getObstacleCenter();
    float getObstacleRight();
};

}
#endif
