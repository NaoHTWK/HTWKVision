#ifndef BALLFEATUREEXTRACTOR_H
#define BALLFEATUREEXTRACTOR_H

#include "base_detector.h"
#include "object_hypothesis.h"

#include <functional>

namespace htwk {

class BallFeatureExtractor : public BaseDetector
{
private:
    static constexpr float FEATURE_SCALE=1.7;
    static constexpr int HIST_SIZE=256;

    int getQ(int *hist, int size, float d);

    void postprocessFeature(const int featureWidth, const int* cnt, int* cyValues, float* dest);

public:
    BallFeatureExtractor(const int _width, const int _height, const int8_t* _lutCb, const int8_t* _lutCr);
    void getFeature(const ObjectHypothesis &p, const uint8_t *img, const int _featureSize, float* dest);
    void getModifiedFeature(const ObjectHypothesis &p, const uint8_t *img, const int featureSize, float* dest, const bool mirrored, const float rotation);
};

} // htwk

#endif // BALLFEATUREEXTRACTOR_H
