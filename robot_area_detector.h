#ifndef __ROBOTAREADETECTOR_H__
#define __ROBOTAREADETECTOR_H__

#include <vector>

#include "robotrect.h"

#include "base_detector.h"
#include "region_classifier.h"
#include "classifier.h"

#include "hypotheses_generator.h"
#include "object_hypothesis.h"
#include "point_2d.h"

namespace htwk {

class RobotAreaDetector : protected BaseDetector
{
private:
    static constexpr int minBorderHeight=8;
    static constexpr float minGreenRatio=0.5;
    int scanlineCnt;

    float pitchRad;
    float rollRad;

    //static constexpr float BALL_SIZE=37.025;//ballradius in px, wenn 1 Meter entfernt
    static constexpr float FEET_SOLO_SIZE=37.025;
    static constexpr float FEET_BOTH_SIZE=45.0;


    std::vector<ObjectHypothesis> hypoList;


    void searchRobotHypotheses(Scanline *scanVertical, const int * const fieldborder);

public:
    RobotAreaDetector(int _width, int _height, int8_t *lutCb, int8_t *lutCr, int scanlineCnt) __attribute__((nonnull));
//    ~RobotAreaDetector() {}

    void proceed(Scanline *scanVertical, const int * const fieldborder, float camPitch, float camRoll) __attribute__((nonnull));
    std::vector<ObjectHypothesis> getHypotheses() const { return hypoList; }
};

}

#endif
