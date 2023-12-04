#ifndef BALLFEATUREEXTRACTOR_H
#define BALLFEATUREEXTRACTOR_H

#include "base_detector.h"
#include "object_hypothesis.h"

namespace htwk {

class BallFeatureExtractor : public BaseDetector {
public:
    using BaseDetector::BaseDetector;

    void getFeature(const ObjectHypothesis& p, const uint8_t* img, int featureSize, float* dest);
    void getFeatureYUV(const ObjectHypothesis& p, const uint8_t* img, const int featureSize, float* dest);
    void getModifiedFeature(const ObjectHypothesis& p, const uint8_t* img, int featureSize, float* dest, bool mirrored,
                            float rotation);

private:
    static constexpr float FEATURE_SCALE = 1.7f;
    static constexpr int HIST_SIZE = 256;

    int getQ(const std::vector<int>& hist, float d);
    void postprocessFeature(std::vector<int>& cnt, std::vector<int>& cyValues, float* dest);
};

}  // namespace htwk

#endif  // BALLFEATUREEXTRACTOR_H
