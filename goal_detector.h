#ifndef __GOAL_DETECTOR_H__
#define __GOAL_DETECTOR_H__

#include <cstdint>
#include <utility>
#include <vector>

#include <color.h>
#include <goalpost.h>
#include <range_check.h>

namespace htwk {

class GoalDetector{
private:
    static const int maxGoalWidth;
    static const int minGoalWidth;
    static const int minGoalHeight;
    static const int scanlineCnt;
    static const int q;
    static const float minWidthToHeightAspect;
    static const int minGoalPostToBorderDist;
    std::vector<GoalPost> goalPosts;
    color goalColor;

    int width, height;
    int *lutCb;
    int *lutCr;

    inline uint8_t getY(const uint8_t * const img,int32_t x,int32_t y) const __attribute__((nonnull)) __attribute__((pure)){
        CHECK_RANGE(x,0,width-1);
        CHECK_RANGE(y,0,height-1);
        return img[(x+y*width)<<1];
    }
    inline uint8_t getCb(const uint8_t * const img,int32_t x,int32_t y) const __attribute__((nonnull)) __attribute__((pure)){
        CHECK_RANGE(x,0,width-1);
        CHECK_RANGE(y,0,height-1);
        return img[((x+y*width)<<1)+lutCb[x]];
    }
    inline uint8_t getCr(const uint8_t * const img,int32_t x,int32_t y) const __attribute__((nonnull)) __attribute__((pure)){
        CHECK_RANGE(x,0,width-1);
        CHECK_RANGE(y,0,height-1);
        return img[((x+y*width)<<1)+lutCr[x]];
    }

public:
    GoalDetector(int width, int height, int *lutCb, int *lutCr);
    ~GoalDetector();

    inline void setY(uint8_t* const img, const int32_t x,int32_t y, const uint8_t c) __attribute__((nonnull)){
        CHECK_RANGE(x,0,width-1);
        CHECK_RANGE(y,0,height-1);
        img[(x+y*width)<<1]=c;
    }

    int findPostYCoord(const uint8_t * const img, color green, color white, int scanlineHeight,
                       int midHeight, float bestM, float bestN, int goalU, int goalV, int breite) const __attribute__((nonnull));

    void proceed(const uint8_t * const img, const int * const fieldborder, color green, color white) __attribute__((nonnull));
    int getGoalPostHeight(const int * const fieldborder) const __attribute__((nonnull));
    std::pair<float,float> calcLine(int scanlineHeight, float stepWidth, int **hist, int f, int* maxPoints,bool &found) __attribute__((nonnull));
    void getMaxPoints(int* maxPoints,int ** hist, int* histSum, int x, int r, int f) const __attribute__((nonnull));
    int getMax(int* hist) const __attribute__((nonnull));
    color getColor() const { return goalColor; };
    void getMaxSup(int* histMax, int* hist, int threshold) const __attribute__((nonnull));
    int **getEdgeHistogramm(const uint8_t * const img,int sy, int ey, float dy) __attribute__((nonnull));
    std::vector<GoalPost> &getGoalPosts();
};

}  // namespace htwk

#endif  // __GOAL_DETECCTOR_H__
