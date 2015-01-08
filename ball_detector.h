#ifndef __BALL_DETECTOR_H__
#define __BALL_DETECTOR_H__

#include <cstdint>

#include <classifier.h>
#include <color.h>
#include <point_2d.h>
#include <range_check.h>

namespace htwk {

struct circle{
    float x,y,r,q;
};

struct Ball{
    int x,y,radius;
    bool found;
};

class BallDetector{

    static const int NUM_STAR_SCANLINES = 16;
    static const int NUM_STAR_SCANLINE_CANDIDATES = 2*NUM_STAR_SCANLINES;

private:
    color ball;

    int ballX;
    int ballY;
    int ballRadius;
    float ballRating;
    int vecLen;
    bool found;

    static const float maxBallRadius;

    static const int diffWhiteCr;
    static const int diffWhiteCy;
    static const int diffGreenCr;
    static const int diffGoalCr;
    static const int minLenBF;
    static const int minLenBW;
    static const float normalizedDistance;

    static const int minColorRating;
    static const int minCy;
    static const int maxCy;
    static const int vecCr;
    static const int vecCb;

    int width, height;
    int *lutCb;
    int *lutCr;
    Classifier* ballClassifier;

    inline uint8_t getY(uint8_t *img,int32_t x,int32_t y) __attribute__((nonnull)) __attribute__((pure)){
        CHECK_RANGE(x,0,width-1);
        CHECK_RANGE(y,0,height-1);
        return img[(x+y*width)<<1];
    }
    inline uint8_t getCb(uint8_t *img,int32_t x,int32_t y) __attribute__((nonnull)) __attribute__((pure)){
        CHECK_RANGE(x,0,width-1);
        CHECK_RANGE(y,0,height-1);
        return img[((x+y*width)<<1)+lutCb[x]];
    }
    inline uint8_t getCr(uint8_t *img,int32_t x,int32_t y) __attribute__((nonnull)) __attribute__((pure)){
        CHECK_RANGE(x,0,width-1);
        CHECK_RANGE(y,0,height-1);
        return img[((x+y*width)<<1)+lutCr[x]];
    }

    static circle ransacCircle(point_2d points[NUM_STAR_SCANLINES], float maxDistEdge);
    static float guessDiameter(point_2d points[NUM_STAR_SCANLINES]);
    static point_2d getInnerPoint(point_2d points[NUM_STAR_SCANLINES], point_2d guess);
    static circle getCircle(float x1,float y1,float x2,float y2,float x3,float y3);
    float classify(point_2d allPoints[NUM_STAR_SCANLINE_CANDIDATES], circle circleA, circle circleB, float diameterGuess, int crDiffGreen, int crDiffWhite, const int * const fieldborder) __attribute__((nonnull));
    void updateBallColor(uint8_t *img, color ball, point_2d *points, int n);
    color getBallColor(uint8_t *img, point_2d guess) __attribute__((nonnull));
    bool raytraceCircle(uint8_t *img, point_2d pos, point_2d points[NUM_STAR_SCANLINES], color ball, color green, color white, color goal) __attribute__((nonnull));
    int getInitialGuess(uint8_t *img, point_2d &maxPos, const int * const fieldborder, const int dy, const int dx, color white) __attribute__((nonnull));
    int getYStartCoord(const int * const fieldborder) const __attribute__((nonnull));

public:

    BallDetector(int width, int height, int *lutCb, int *lutCr) __attribute__((nonnull));


    inline void setY(uint8_t* const img, const int32_t x,int32_t y, const uint8_t c) __attribute__((nonnull)){
        CHECK_RANGE(x,0,width-1);
        CHECK_RANGE(y,0,height-1);
        img[(x+y*width)<<1]=c;
    }

    void proceed(uint8_t *img, const int * const fieldborder, color green, color white, color goal) __attribute__((nonnull));
    int getBallX() const { return ballX; };
    int getBallY() const { return ballY; };
//	int getBallRadius(float alpha,float y);
    int getBallRadius() const;
    float getBallRating() const { return ballRating; };
    bool isBallFound() const { return found; };
    bool isBall(int x, int y) const ;
    Ball getBall();
    color getColor() const { return ball; };

};

}  // namespace htwk

#endif  // __BALL_DETECTOR_H__
