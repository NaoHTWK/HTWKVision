#ifndef __ROBOT_AREA_DETECTOR_H__
#define __ROBOT_AREA_DETECTOR_H__

#include <cstdint>
#include <vector>

#include <ball_detector.h>
#include <field_color_detector.h>
#include <goalpost.h>
#include <rect.h>
#include <region_classifier.h>

namespace htwk {

class RobotAreaDetector{
private:
    RobotAreaDetector(RobotAreaDetector& l);
    void operator=(RobotAreaDetector const&);

    int width;
    int height;
    int minRobotRegions;
    int minRobotHeight;
    int minRobotWidth;
    int minRobotArea;
    int  scanlineCnt;
    int *robotAreaYTop;
    int *robotAreaYBottom;
    int *robotAreaYTmp;
    std::vector<Rect> *robotAreas;

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
    inline void setY(uint8_t * const img,int32_t x,int32_t y, uint8_t c) const __attribute__((nonnull)) {
        CHECK_RANGE(x,0,width-1);
        CHECK_RANGE(y,0,height-1);
        img[(x+y*width)<<1]=c;
    }

    RobotAreaDetector(int width, int height, int *lutCb, int *lutCr, int scanlineCnt) __attribute__((nonnull));
    ~RobotAreaDetector();

    static bool isBall(int x, int y, const Ball &ball);
    static bool isInGoal(std::vector<GoalPost> goalPosts, Rect r);

    void proceed(const uint8_t * const img, Scanline *scanVertical, const int * const fieldborder,
                FieldColorDetector *field, const Ball &ball, const std::vector<GoalPost> &goalPosts) __attribute__((nonnull));
    void morphMax(int *data);
    void morphMin(int *data);
    void searchRobotAreas(Scanline *scanVertical);
    int searchRobotBorder(const int * const fieldborder, const uint8_t * const img, FieldColorDetector *field,
                          const Ball &ball, Rect r, int xStart, int dx) const;
    std::vector<Rect> *getRobotAreas();
};

}  // namespace htwk

#endif  // __ROBOT_AREA_DETECTOR_H__
