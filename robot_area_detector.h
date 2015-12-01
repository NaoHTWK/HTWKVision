#ifndef __ROBOTAREADETECTOR_H__
#define __ROBOTAREADETECTOR_H__

#include <stdint.h>
#include <x86intrin.h>

#include <vector>

#include <robotrect.h>
#include <goalpost.h>

#include <ball_detector.h>
#include <base_detector.h>
#include <region_classifier.h>
#include <classifier.h>

namespace htwk {

class RobotAreaDetector : protected BaseDetector
{
private:
    static constexpr int minBorderHeight=8;
    static constexpr float minGreenRatio=0.5;
    static constexpr int fieldMapScale=2;
    RobotAreaDetector(RobotAreaDetector& l);
    void operator=(RobotAreaDetector const&);
	int scanlineCnt;
	int fieldMapWidth;
	int fieldMapHeight;
    int fieldMapWidthSSE;
    uint8_t *fieldMap;
	int *fieldMapIntegral;
    std::vector<RobotRect> *robotAreas;

    __m128i* yout;
    __m128i* uout;
    __m128i* vout;

    struct Boundries {
        __m128i minCy;
        __m128i minCb;
        __m128i minCr;
        __m128i maxCy;
        __m128i maxCb;
        __m128i maxCr;
    };

    void createGreenMapSSEBatch(const uint8_t * const img, FieldColorDetector *field, int x, int y);
    void createGreenMapSSECompare(Boundries& b, int x, int y);
    void createGreenMap(const int * const fieldborder, const uint8_t * const img, FieldColorDetector *field);
    void createGreenMapSSE(const int * const fieldborder, const uint8_t * const img, FieldColorDetector *field);
    void createIntegralImage();
    float getIntegralValue(int px1, int py1, int px2, int py2);

public:
    RobotAreaDetector(int width, int height, int8_t *lutCb, int8_t *lutCr, int scanlineCnt) __attribute__((nonnull));
    ~RobotAreaDetector() {}

    double proceed(uint8_t * const img, Scanline *scanVertical, const int * const fieldborder,
                FieldColorDetector *field, const Ball &ball, const std::vector<GoalPost> &goalPosts) __attribute__((nonnull));
    std::vector<RobotRect> searchRobotHypotheses(Scanline *scanVertical, const int * const fieldborder);
    std::vector<RobotRect> setCarpetFeatures(std::vector<RobotRect> hypotheses);

    std::vector<RobotRect> *getRobotAreas();
};

}

#endif
