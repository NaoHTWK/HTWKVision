#ifndef __BALL_DETECTOR_II_H__
#define __BALL_DETECTOR_II_H__

#include <cstdint>
#include <memory>
#include <vector>

#include <base_detector.h>
#include <ball_detector.h>
#include <classifier.h>
#include <color.h>
#include <point_2d.h>
#include <range_check.h>
#include <field_color_detector.h>

namespace htwk {

class BallDetectorII : protected BaseDetector {

private:

    int INTEGRAL_SCALE=2;//only 1,2 or 4
    int BLOCK_SIZE=32;
    int MIN_OBJECT_RADIUS=7;
    int MAX_NUM_HYPOTHESES=9;
    int MIN_RATING=300;
    int FEATURE_RADIUS=7;
    double FEATURE_SCALE=1.5;
    double MIN_PROB=0.125;
    double SCALE_SMALL=0.8956;//Optimizer.getParam("R1",0.7,1.0);
    double SCALE_CENTER=SCALE_SMALL+0.1597;//Optimizer.getParam("dR2",0.05,0.4);
    double SCALE_BIG=SCALE_CENTER+0.2967;//Optimizer.getParam("dR3",0.05,0.4);
    int CAM_WIDTH=640;
    int CAM_HEIGHT=480;
    double CAM_FOV=1.03;
    double f=CAM_WIDTH/2./tan(CAM_FOV/2);
    double BOT_HEIGHT=0.4694;//Optimizer.getParam("botHeight",0.35,0.55);
    double BALL_SIZE=30.025;//Optimizer.getParam("ballSize",22,38);//ballradius in px, wenn 1 Meter entfernt

    double rollRad;
    double pitchRad;
    int iWidth;
    int iHeight;

    int blockSize=BLOCK_SIZE/INTEGRAL_SCALE;
    int numBlockX;
    int numBlockY;
    int *integralImg;
    int *ratingImg;
    int *blockMeanValues;
    int *blockMaxX;
    int *blockMaxY;
    int *isBlockUsable;
    int *blockObjectRadius;

    //vector<ObjectHypothesis> ballHypotheses;

    Classifier* classifier;
    int ballX;
    int ballY;
    int ballRadius;
    float ballRating;
    bool found;

    color ballColor;

public:

    BallDetectorII(int width, int height, int8_t *lutCb, int8_t *lutCr) __attribute__((nonnull));
    void proceed(uint8_t *img, const int * const fieldborder, const FieldColorDetector *const field) __attribute__((nonnull));
    void setBlockValues(uint8_t *img, const FieldColorDetector *const fieldColorDetector) const __attribute__((nonnull));
    void createIntegralImage(uint8_t *img) const __attribute__((nonnull));
    int getArea(int px1, int py1, int px2, int py2) const __attribute__((nonnull));
    int getIntegralValue(int x1, int y1, int x2, int y2) const __attribute__((nonnull));
    int getBallX() const { return ballX; }
    int getBallY() const { return ballY; }
    int getBallRadius() const;
    float getBallRating() const { return ballRating; }
    bool isBallFound() const { return found; }
    bool isBall(int x, int y) const ;
    Ball getBall();
    color getColor() const { return ballColor; }
};

}  // namespace htwk

#endif  // __BALL_DETECTOR_II_H__
