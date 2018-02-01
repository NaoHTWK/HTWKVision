#ifndef __FEET_DETECTOR_H__
#define __FEET_DETECTOR_H__


#include <cstdint>
#include <vector>

#include "base_detector.h"
#include "field_color_detector.h"
#include "point_2d.h"
#include "color.h"
#include "exponential_moving_average.h"

namespace htwk {

class FeetDetector : protected BaseDetector {
private:
    static constexpr int q=4;
    int *feetBorder;
    int *footPattern;
    int patternWidth;
    int patternHeight;
    float minRating;
    int feetX;
    int feetY;
    bool found;
    EMA avgX;
    EMA avgY;
    bool avgAvailable;

    float *feetRating;

    int *feetBorderRaw;
    float *feetRatingRaw;

    /*This class souldn't be copied */
    FeetDetector() = delete;
    FeetDetector(FeetDetector& h) = delete;
    void operator=(FeetDetector const&) = delete;

    void createFootPattern();

public:
    FeetDetector(int _width, int _height, int8_t * _lutCb, int8_t * _lutCr) __attribute__((nonnull));
    ~FeetDetector();
    void proceed(const uint8_t * const img, FieldColorDetector *field, color ball, bool ballFound, bool useDetection) __attribute__((nonnull));
    bool isAvailable() const;
    bool isInterpolated() const;
    point_2d getBase() const;
};

}  // namespace htwk

#endif  // __FEET_DETECTOR_H__
