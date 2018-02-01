#ifndef __RANSAC_ELLIPSE_FITTER_H__
#define __RANSAC_ELLIPSE_FITTER_H__

#include <random>
#include <vector>

#include "ellipse.h"
#include "linesegment.h"
#include "point_2d.h"

namespace htwk {

class RansacEllipseFitter {
private:
    static const float minRating;
    static const int angleCnt;
    bool ellipseFound;
    Ellipse resultEllipse;
    float camPitch;
    float camRoll;
    std::mt19937 rng;
    std::uniform_real_distribution<float> dist{0,1};

public:
    RansacEllipseFitter();
    virtual ~RansacEllipseFitter();
    static void eigenvectors(float a, float b, float eva[2],float eve[][2]);
    static void eigenvalues(float a, float b, float c,float erg[2]);
    static int det(float a[][2]);
    static void transformPo(point_2d &p, const float trans[][2], const float translation[] );
    static void transformPoInv(point_2d &p, const float trans[][2], const float translation[2] );
    static float getEllDist(float px, float py, Ellipse trEl);
    static int transformEl(Ellipse &el);

    void proceed(const std::vector<LineSegment *> &lineEdgeSegments);
    float getRating(const std::vector<LineSegment *> &carryover, const Ellipse &e);
    float ransacFit(const std::vector<LineSegment *> &carryover,
                    const std::vector<LineSegment *> &lineEdgeSegments, float ellipse[6],
                    int iter, unsigned int minMatches);
    Ellipse &getEllipse();
    float getCamPitch() const { return camPitch; }
    float getCamRoll()  const { return camRoll;  }
};

}  // namespace htwk

#endif  // __RANSAC_ELLIPSE_FITTER_H__
