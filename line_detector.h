#ifndef __LINE_DETECTOR_H__
#define __LINE_DETECTOR_H__

#include <cstdlib>
#include <cstdint>
#include <vector>

#include "base_detector.h"
#include "color.h"
#include "linecross.h"
#include "lineedge.h"
#include "linegroup.h"
#include "linesegment.h"
#include "point_2d.h"
#include "range_check.h"

namespace htwk {

class LineDetector : protected BaseDetector {
private:

public:
	static const size_t minSegmentCnt;
    static const float maxError;
    std::vector<LineEdge*> linesTmp;
    std::vector<LineEdge*> lineEdges;
    std::vector<LineGroup> linesList;
    std::vector<LineCross> crossings;
	color white;

    LineDetector(int width, int height, int8_t *lutCb, int8_t *lutCr) __attribute__((nonnull));
	~LineDetector();

    static bool isStraight(LineEdge *line) __attribute__((nonnull));
	static float getError(LineSegment *le1, LineSegment *le2) __attribute__((nonnull));
	static float getError2(LineSegment *le1, LineSegment *le2) __attribute__((nonnull));

    void proceed(uint8_t *img, std::vector<LineSegment*> lineSegments, int q) __attribute__((nonnull));
	point_2d getIntersection(float px1, float py1, float vx1, float vy1, float px2, float py2, float vx2, float vy2);
    LineEdge createLineEdge(std::vector<LineSegment*> segments);
    void updateWhiteColor(std::vector<LineSegment*> lineSegments, uint8_t *img) __attribute__((nonnull));
	void findLineGroups();
    std::vector<LineGroup> &getLineGroups();
    color getColor() const { return white; }
};

}  // namespace htwk

#endif  // __LINE_DETECTOR_H__
