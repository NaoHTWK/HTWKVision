#ifndef __ROBOT_CLASSIFIER_H__
#define __ROBOT_CLASSIFIER_H__

#include <cstdint>

#include <vector>

#include "base_detector.h"
#include "classifier.h"
#include "rect.h"
#include "robotrect.h"
#include "team_membership.h"

#define HISTOGRAMM_SHIFT 2
#define NORMWIDTH 17
#define NORMHEIGHT 24
#define HISTOGRAM_SHIFT_GRADIENT 2
#define HISTOGRAM_SHIFT_COLOR 2

namespace htwk {

struct RobotClassifierResult{
    Rect rect;
    float detectionProbability;
    struct {
        float entropy_Y;
        float entropy_Y_signed;
        float entropy_X_signed;
        float entropy_cl_Y_channel;
        float color_mean_squared_deviation_Y_channel;
        float entropy_linesum_X;
        float entropy_linesum_X_signed;
        float entropy_linesum_Y_signed;
    } features;
    float isBlueJersey;
    TeamMembership teamColor;
};

class RobotClassifier : protected BaseDetector
{
private:
    struct ycbcr32_t{
        int32_t y;
        int32_t cb;
        int32_t cr;
    };
    Classifier *robotClassifierNN;
    int histolength;

    static void fill(int * a, int val, int max);
    static float getGradientEntropy_Y_Direction(float* hist, int histlength);
    static float getColorEntropy(float** hist, int length);
    static int normSum_Y_Line(int x,const ycbcr32_t *normImage);
    static int normSum_X_Line(int y,const ycbcr32_t *normImage);
    static float processEntropy(float* grad_hist, int length);
    static float calcColorMean(const ycbcr32_t *img);
    static void norm(float *m,int numFeatures);
    static float calcMeanDeviation(const ycbcr32_t * const img);

    void generateNormImage(const uint8_t * const img, RobotRect &r, ycbcr32_t * const normImage) const __attribute__((nonnull));
    float* getGradientHistogramm_Y_Direction(const ycbcr32_t *normImage);
    float* normHistogram(int* histo, int cnt, int histolength);
    float** getColorHistograms(const ycbcr32_t *normImage);
    float** normHistogram(int** histo, int cnt, int histolength);
    float getGradientEntropy_normLineSum_X_Direction(const ycbcr32_t *normImage) ;
    float getGradientEntropy_normLineSum_X_Direction_Signed(const ycbcr32_t *normImage);
    float *getBackTheSignedHist(float* hist, int length);
    float getGradientEntropySigned_Y_Direction(const ycbcr32_t *normImage);
    float getGradientEntropySigned_X_Direction(const ycbcr32_t *normImage);
    float* getGradientHistogramm_X_Direction(const ycbcr32_t *normImage);
    float getGradientEntropy_normLineSum_Y_Direction_Signed(const ycbcr32_t *normImage);


    TeamMembership determineTeamColor(const uint8_t * const img,Rect r) const;

public:
    RobotClassifier(int width, int height, int8_t *lutCb, int8_t *lutCr) __attribute__((nonnull));
    ~RobotClassifier() {}
    void proceed(const uint8_t * const img, RobotRect& r, RobotClassifierResult &result) __attribute__((nonnull));
    float isRobot(RobotClassifierResult result, const RobotRect &r);
};

}  // namespace htwk

#endif  // __ROBOT_CLASSIFIER_H__
