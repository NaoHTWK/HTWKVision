#ifndef __GOAL_DETECTOR_H__
#define __GOAL_DETECTOR_H__

#include <cstdint>
#include <utility>
#include <vector>

#include "base_detector.h"
#include "classifier.h"
#include "color.h"
#include "goalpost.h"
#include "point_2d.h"

namespace htwk {

class GoalDetector : public BaseDetector {
public:
    GoalDetector(int width, int height, int8_t *lutCb, int8_t *lutCr);
    void proceed(uint8_t *img, const int * const fieldborder, color green, color white);
    std::vector<GoalPost> getGoalPosts();
    std::vector<GoalPost> goalPosts;
    color getColor() { return goalColor; }

private:
    void analyseGoalPost(uint8_t *img,
                         int xLeft,
                         int xRight,
                         const int * const fieldborder,
                         color green);
    float getGreenFeature(uint8_t *img,
                          int baseX,
                          int baseY,
                          int length,
                          int whiteY,
                          int whiteCb,
                          int whiteCr,
                          color green,
                          int vx,
                          int vy,
                          int steps);
    int searchBase(uint8_t *img,
                   int lowerY,
                   int baseSearchHeight,
                   float a1,
                   float a0,
                   int whiteY,
                   int whiteCb,
                   int whiteCr,
                   color green);
    int findMaxWhite(uint8_t *img,
                     point_2d &base,
                     int windowSize,
                     int windowSteps,
                     int searchSteps);

    void getEdgeTable(uint8_t *img,
                                  const int * const fieldborder,
                                  int scanlineCnt, std::vector<int>& hist);

    static const int maxGoalWidth;
    static const int minGoalWidth;
    static const int minGoalHeight;
    static const int scanlineCnt;
    static const int q;
    static const int minGoalPostToBorderDist;
    static const bool detectDarkGoals;
    Classifier* goalClassifier;

    color goalColor;
    float bestProbability;
};
}  // namespace htwk

#endif  // __GOAL_DETECCTOR_H__

