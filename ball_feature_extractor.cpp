#include "ball_feature_extractor.h"

#include <algorithm_ext.h>

#include <algorithm>
#include <cmath>
#include <cstring>

namespace htwk {

#define fast_round(x) (((int)((x) + 100.5f)) - 100)

void BallFeatureExtractor::getFeature(const ObjectHypothesis& p, const uint8_t* img, const int featureSize,
                                      float* dest) {
    const float scale = 0.5f * p.r * FEATURE_SCALE / (featureSize / 2);

    std::vector<int> cyValues(featureSize * featureSize, 0);
    std::vector<int> cnt(featureSize * featureSize, 0);

    for (int dy = -featureSize; dy < featureSize; dy++) {
        int y = p.y + fast_round(dy * scale);
        if (y < 0 || y >= height)
            continue;
        int addr_y = ((dy + featureSize) / 2) * featureSize;
        for (int dx = -featureSize; dx < featureSize; dx++) {
            int x = p.x + fast_round(dx * scale);
            if (x < 0 || x >= width) {
                continue;
            }
            int addr = ((dx + featureSize) / 2) + addr_y;
            cnt[addr]++;
            cyValues[addr] += getY(img, x, y);
        }
    }

    postprocessFeature(cnt, cyValues, dest);
}

void BallFeatureExtractor::getFeatureYUV(const ObjectHypothesis& p, const uint8_t* img, const int featureSize,
                                         float* dest) {
    const float scale = 0.5f * p.r * FEATURE_SCALE / (featureSize / 2);

    std::vector<int> values(featureSize * featureSize * 3, 0);
    std::vector<int> cnt(featureSize * featureSize, 0);

    for (int dy = -featureSize; dy < featureSize; dy++) {
        int y = p.y + fast_round(dy * scale);
        if (y < 0 || y >= height)
            continue;
        int addr_y = ((dy + featureSize) / 2) * featureSize * 3;
        int addr_cnt_y = ((dy + featureSize) / 2) * featureSize;

        for (int dx = -featureSize; dx < featureSize; dx++) {
            int x = p.x + fast_round(dx * scale);
            if (x < 0 || x >= width) {
                continue;
            }
            int addr_yuv = ((dx + featureSize) / 2) * 3 + addr_y;
            int addr_cnt = ((dx + featureSize) / 2) + addr_cnt_y;
            cnt[addr_cnt]++;
            values[0 + addr_yuv] += getY(img, x, y);
            values[1 + addr_yuv] += getCb(img, x, y);
            values[2 + addr_yuv] += getCr(img, x, y);
        }
    }

    std::vector<float> fvalues(std::begin(values), std::end(values));
    for (int y = 0; y < featureSize; y++) {
        for (int x = 0; x < featureSize; x++) {
            int idx_yuv = x * 3 + (y * featureSize * 3);
            int idx_cnt = x + (y * featureSize);
            if (cnt[idx_cnt] == 0) {
                fvalues[0 + idx_yuv] = 0;
                fvalues[1 + idx_yuv] = 0;
                fvalues[2 + idx_yuv] = 0;
                continue;
            }
            fvalues[0 + idx_yuv] /= cnt[idx_cnt] / 255.f;
            fvalues[1 + idx_yuv] /= cnt[idx_cnt] / 255.f;
            fvalues[2 + idx_yuv] /= cnt[idx_cnt] / 255.f;
        }
    }
    memcpy(dest, fvalues.data(), fvalues.size() * sizeof(float));
}

void BallFeatureExtractor::getModifiedFeature(const ObjectHypothesis& p, const uint8_t* img, const int featureSize,
                                              float* dest, const bool mirrored, const float rotation) {
    const float scale = p.r * FEATURE_SCALE / (featureSize / 2);

    std::vector<int> cyValues(featureSize * featureSize, 0);
    std::vector<int> cnt(featureSize * featureSize, 0);

    for (int dy = -featureSize; dy < featureSize; dy++) {
        for (int dx = -featureSize; dx < featureSize; dx++) {
            float vx = dx * cosf(rotation) - dy * sinf(rotation);
            float vy = dx * sinf(rotation) + dy * cosf(rotation);
            int x = p.x + fast_round((mirrored ? -1 : 1) * vx * scale * 0.5f);
            int y = p.y + fast_round(vy * scale * 0.5f);
            if (x < 0 || y < 0 || x >= width || y >= height) {
                continue;
            }
            size_t addr = ((dx + featureSize) / 2) + ((dy + featureSize) / 2) * featureSize;
            cnt[addr]++;
            cyValues[addr] += getY(img, x, y);
        }
    }

    postprocessFeature(cnt, cyValues, dest);
}

void BallFeatureExtractor::postprocessFeature(std::vector<int>& cnt, std::vector<int>& cyValues, float* dest) {
//    int meanCnt = 0;
    std::vector<int> hist(HIST_SIZE, 0);
    for (size_t i = 0; i < cyValues.size(); i++) {
        if (cnt[i] < 1)
            continue;
        cyValues[i] /= cnt[i];
        hist[cyValues[i]]++;
//        meanCnt++;
    }

    float qMin = getQ(hist, 0.05f);
    float qMax = getQ(hist, 0.95f);
    float meanCy = (qMin + qMax) * 0.5f;
    float var = (qMax - qMin) * 0.25f;

    float varInv = 32.f / (32 + var);
    for (size_t i = 0; i < cyValues.size(); i++) {
        dest[i] = (cnt[i] < 1) ? 0.f : (cyValues[i] - meanCy) * varInv;
    }
}

int BallFeatureExtractor::getQ(const std::vector<int>& hist, float d) {
    int sum = accumulate(hist) * d;
    int sumQ = 0;
    for (size_t i = 0; i < hist.size(); i++) {
        sumQ += hist[i];
        if (sumQ >= sum) {
            return i;
        }
    }
    return 0;
}

}  // namespace htwk
