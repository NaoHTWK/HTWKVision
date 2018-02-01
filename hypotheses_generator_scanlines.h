#ifndef HYPOTHESES_GENERATOR_SCANLINES_H
#define HYPOTHESES_GENERATOR_SCANLINES_H

#include <base_detector.h>
#include <hypotheses_generator.h>
#include <htwk_vision_config.h>
#include <point_2d.h>
#include <line.h>

#include <cmath>

namespace htwk {

#define maxEdges 50
//EdgeX-Y -> Bottom of an scanline segment
struct RatingScanline{
    int edgeCnt;
    int edgesX[maxEdges];
    int edgesY[maxEdges];
    bool regionsIsObj[maxEdges];
};

class HypothesesGeneratorScanlines : public HypothesesGenerator, protected BaseDetector
{
private:
    static constexpr int RATING_SCALE=4;                    //only 1,2 or 4
    static const float MIN_OBJECT_RADIUS_NORM;
    static constexpr float borderCompensation=5;            //lock some pixels above field border for hypotheses
    static constexpr float BALL_SIZE=37.025f;
    static constexpr int numAngles=8;                       //number of points for circle pattern
    static constexpr int BLOCK_SIZE=32;                     //divide image into blocks and calculate object size only block-wise
    static const int blockSize;
    static constexpr float OBJECT_RADIUS_SCALE=1.5f;
    size_t MAX_NUM_HYPOTHESES=16;                              //max number of hypotheses generated

    static constexpr bool PRUNE_HYPOTHESIS_OVER_HORIZON = true;

    std::vector<ObjectHypothesis> hypoList;  // list with hypotheses results
    std::vector<point_2d> vec;               //vector with circle-pattern for position improvement

    IntegralImage* integralImg;

    const int rWidth;
    const int rHeight;
    const int numBlockX;
    const int numBlockY;
    float rollRad;
    float pitchRad;
    std::vector<float> blockObjectRadius;
    int* ratingImg;

    const int lineSpace;
    int scale;
    RatingScanline* scanVertical;
    RatingScanline* scanHorizontal;
    int histCnt;
    std::array<int,256> histData;
    int threshold;

    void improvePositionAccuracy(std::vector<ObjectHypothesis>& selectedList);
    void improveHypothesesRatings(std::vector<ObjectHypothesis>& selectedList);
    void createRatingImg(const int* const border);
    void calcOtsuThreshold();
    void selectBestHypotheses(std::vector<ObjectHypothesis>& maxList);
    bool containsObject(const std::vector<ObjectHypothesis>& maxList, const ObjectHypothesis& hyp);
    void calculateBlockBlur(const float objectRadius, const int px, const int py, const int* const border);
    void calculateBlockBlurSSERatingScale2(const float objectRadius, const int px, const int py);
    void calculateBlockBlurSSERatingScale4(const float objectRadius, const int px, const int py);
    void calculateBlockBlurBorder(const float objectRadius, const int px, const int py);

    void calculateBlockRadii();

    // integral image getArea function
    inline int getArea(const int px1, const int py1, const int px2, const int py2) { return (px2-px1)*(py2-py1); }

    void createIntegralImage(int* dataY, int* dataCr, int* iImage, int iWidth, int iHeight);
    point_2d getSubpixelPosition2(const int* data, int x, int y, const int width, const int height);

    void scanRatingImg(const int* border);
    void analyseScanlinesHorizontal(const Line &horizon);
    void analyseScanlinesVertical(const htwk::Line &horizon);
    int calcRatingCross(int left, int right, int top, int bottom, int centerX, int centerY);
    int calcRatingIntegral(int left, int right, int top, int bottom, int size);
    int calcPoints(int left, int right, int top, int bottom, int centerX, int centerY);
    int calcCrossPoints(int left, int right, int top, int bottom, int centerX, int centerY);

public:
    const int svMax;
    const int shMax;

    RatingScanline* getScanVertical() { return scanVertical; }
    RatingScanline* getScanHorizontal() { return scanHorizontal; }

    HypothesesGeneratorScanlines(int width, int height, int8_t* lutCb, int8_t* lutCr, const HtwkVisionConfig& config);
    ~HypothesesGeneratorScanlines() override;

    void proceed(uint8_t* img, const int * const fieldBorder, float camPitch, float camRoll, IntegralImage* _integralImg) override;
    std::vector<ObjectHypothesis> getHypotheses() const override { return hypoList; }
    int* getRatingImg() override { return ratingImg; }
};

} // htwk

#endif // HYPOTHESES_GENERATOR_SCANLINES_H
