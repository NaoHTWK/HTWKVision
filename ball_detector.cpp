#include "ball_detector.h"

namespace htwk {

BallDetector::BallDetector(const int width, const int height,
                           const int8_t *lutCb, const int8_t *lutCr)
    : BaseDetector(width, height, lutCb, lutCr)
{
}

BallDetector::~BallDetector() {}

}
