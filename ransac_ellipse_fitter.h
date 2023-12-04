#ifndef __RANSAC_ELLIPSE_FITTER_H__
#define __RANSAC_ELLIPSE_FITTER_H__

#include <random>
#include <vector>

#include "base_detector.h"
#include "ellipse.h"
#include "htwk_vision_config.h"
#include "linesegment.h"
#include "point_2d.h"

namespace htwk {

class RansacEllipseFitter : public BaseDetector {
private:
    static float minRating;
    static int angleCnt;
    static int minCurvedSegments;
    static int minIterationTries;

    float minDistanceFromEllipse = 0.05f;
    int sucessfullDetectionWithAdditionLineSegments = 0;
    int sucessfullDetectionWithoutAdditionalLineSigments = 0;

    bool ellipseFound{false};
    Ellipse resultEllipse;
    std::mt19937 rng;
    std::uniform_real_distribution<float> dist{0,1};
    int abortState{0};

public:
    RansacEllipseFitter(const int8_t* lutCb, const int8_t* lutCr, HtwkVisionConfig& config);
    RansacEllipseFitter(const RansacEllipseFitter&) = delete;
    RansacEllipseFitter(RansacEllipseFitter&&) = delete;
    RansacEllipseFitter& operator=(const RansacEllipseFitter&) = delete;
    RansacEllipseFitter& operator=(RansacEllipseFitter&&) = delete;
    ~RansacEllipseFitter() = default;

    static void eigenvectors(float a, float b, const float eva[2],float eve[][2]);
    static void eigenvalues(float a, float b, float c,float erg[2]);
    static int det(float a[][2]);
    static void transformPo(point_2d &p, const float trans[][2], const float translation[2] );
    static void transformPoInv(point_2d &p, const float trans[][2], const float translation[2] );
    static float getEllDist(float px, float py, Ellipse trEl);
    static int transformEl(Ellipse &el);

    void proceed(const std::vector<LineSegment *> &lineEdgeSegments, uint8_t *image);
    float getRating(const std::vector<LineSegment *> &carryover, const Ellipse &e);
    float ransacFit(const std::vector<LineSegment *> &carryover,
                    const std::vector<LineSegment *> &lineEdgeSegments, float ellipse[6],
                    int iter, unsigned int minMatches, uint8_t *image, float &iterMaxRating);
    Ellipse &getEllipse();
};

}  // namespace htwk

#endif  // __RANSAC_ELLIPSE_FITTER_H__
