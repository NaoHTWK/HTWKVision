#ifndef __ROBOT_CLASSIFIER_H__
#define __ROBOT_CLASSIFIER_H__

#include <cstdint>

#include <vector>

#include <classifier.h>
#include <range_check.h>
#include <rect.h>

#define HISTOGRAMM_SHIFT 2
#define NORMWIDTH 17
#define NORMHEIGHT 24
#define HISTOGRAM_SHIFT_GRADIENT 2
#define HISTOGRAM_SHIFT_COLOR 2

namespace htwk {

enum class TeamMembership {
    NONE,
    BLUE,
    RED
};

struct classifierResult{
    Rect rect;
    bool isRobot;
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
    TeamMembership teamColor;
};

class RobotClassifier {
private:
    struct ycbcr32_t{
        int32_t y;
        int32_t cb;
        int32_t cr;
    };
    Classifier *robotClassifierNN;
    int width;
    int height;
    int *lutCb;
    int *lutCr;
    int histolength;
    inline uint8_t getY(const uint8_t * const img, int32_t x, int32_t y) const
    __attribute__((nonnull)) __attribute__((pure)) {
        CHECK_RANGE(x,0,width-1);
        CHECK_RANGE(y,0,height-1);
        return img[(x + y * width) << 1];
    }
    inline uint8_t getCb(const uint8_t * const img, int32_t x, int32_t y) const
    __attribute__((nonnull)) __attribute__((pure)) {
        CHECK_RANGE(x,0,width-1);
        CHECK_RANGE(y,0,height-1);
        return img[((x + y * width) << 1) + lutCb[x]];
    }
    inline uint8_t getCr(const uint8_t * const img, int32_t x, int32_t y) const
    __attribute__((nonnull)) __attribute__((pure)) {
        CHECK_RANGE(x,0,width-1);
        CHECK_RANGE(y,0,height-1);
        return img[((x + y * width) << 1) + lutCr[x]];
    }
    static void fill(int * a, int i, int max);
    static float getGradientEntropy_Y_Direction(float* hist, int histlength);
    static float getColorEntropy(float** hist, int length);
    static int normSum_Y_Line(int x,const ycbcr32_t *normImage);
    static int normSum_X_Line(int y,const ycbcr32_t *normImage);
    static float processEntropy(float* grad_hist, int length);
    static float calcColorMean(const ycbcr32_t *img);
    static void norm(float *m,int numFeatures);
    static float calcMeanDeviation(const ycbcr32_t * const img);

    void generateNormImage(const uint8_t * const img, Rect r,
            ycbcr32_t * const normImage) const __attribute__((nonnull));
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
    inline void setY(uint8_t * const img, int32_t x, int32_t y, uint8_t c) const
    __attribute__((nonnull)) {
        CHECK_RANGE(x,0,width-1);
        CHECK_RANGE(y,0,height-1);
        img[(x + y * width) << 1] = c;
    }

    RobotClassifier(int width, int height, int *lutCb, int *lutCr) __attribute__((nonnull));
    ~RobotClassifier();
    void proceed(const uint8_t * const img, Rect r, classifierResult &result) __attribute__((nonnull));
    bool isRobot(classifierResult result);
};

}  // namespace htwk

#endif  // __ROBOT_CLASSIFIER_H__
