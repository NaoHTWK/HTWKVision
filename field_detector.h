#ifndef __FIELD_DETECTOR_H__
#define __FIELD_DETECTOR_H__

#include <random>

#include "field_color_detector.h"

#include <base_detector.h>
#include <line.h>
#include <region_classifier.h>

namespace htwk {

struct Region {
    int xLeft, xRight, yTop, yBottom;
    int size;
    bool isField;
};

class FieldDetector : public BaseDetector {
private:
    std::mt19937 rng;
    std::uniform_real_distribution<> dist{0,1};

    int *fieldBorderFull;

    FieldDetector();
    FieldDetector(const FieldDetector &cpy);
    FieldDetector operator=(FieldDetector &f);

    std::vector<Line> fieldBorderLines;

public:
    FieldDetector(int width, int height, int8_t *lutCb, int8_t *lutCr) __attribute__((nonnull));
    ~FieldDetector();
    void proceed(const uint8_t *const img, const FieldColorDetector *const field,
                 const RegionClassifier *const regionClassifier)
    __attribute__((nonnull));

    const std::vector<Line>& getFieldBorderLines() const {
        return fieldBorderLines;
    }

    const int* getConvexFieldBorder() const {
        return fieldBorderFull;
    }

};

}  // namespace htwk
#endif  // __FIELD_DETECTOR_H__
