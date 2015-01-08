#ifndef __LINE_DETECTOR_H__
#define __LINE_DETECTOR_H__

#include <cstdlib>
#include <stdint.h>
#include <vector>

#include <color.h>
#include <linecross.h>
#include <lineedge.h>
#include <linegroup.h>
#include <linesegment.h>
#include <point_2d.h>
#include <range_check.h>

namespace htwk {

class LineDetector{
private:

    int width;
    int height;
	int *lutCb;
	int *lutCr;

    inline uint8_t getY(uint8_t *img,int32_t x,int32_t y) const __attribute__((nonnull)) __attribute__((pure)){
		CHECK_RANGE(x,0,width-1);
		CHECK_RANGE(y,0,height-1);
		return img[(x+y*width)<<1];
	}
	inline uint8_t getCb(uint8_t *img,int32_t x,int32_t y) const __attribute__((nonnull)) __attribute__((pure)){
		CHECK_RANGE(x,0,width-1);
		CHECK_RANGE(y,0,height-1);
		return img[((x+y*width)<<1)+lutCb[x]];
	}
	inline uint8_t getCr(uint8_t *img,int32_t x,int32_t y) const __attribute__((nonnull)) __attribute__((pure)){
		CHECK_RANGE(x,0,width-1);
		CHECK_RANGE(y,0,height-1);
		return img[((x+y*width)<<1)+lutCr[x]];
	}

public:
	static const size_t minSegmentCnt;
	static const double maxError;
    std::vector<LineEdge*> linesTmp;
    std::vector<LineEdge*> lineEdges;
    std::vector<LineGroup> linesList;
    std::vector<LineCross> crossings;
	color white;

    LineDetector(int width, int height, int *lutCb, int *lutCr) __attribute__((nonnull));
	~LineDetector();

    inline void setY(uint8_t* const img, const int32_t x,int32_t y, const uint8_t c) __attribute__((nonnull)){
        CHECK_RANGE(x,0,width-1);
        CHECK_RANGE(y,0,height-1);
        img[(x+y*width)<<1]=c;
    }

    static bool isStraight(LineEdge *line) __attribute__((nonnull));
	static float getError(LineSegment *le1, LineSegment *le2) __attribute__((nonnull));
	static float getError2(LineSegment *le1, LineSegment *le2) __attribute__((nonnull));

    void proceed(uint8_t *img, std::vector<LineSegment*> lineSegments, int q) __attribute__((nonnull));
	point_2d getIntersection(float px1, float py1, float vx1, float vy1, float px2, float py2, float vx2, float vy2);
    LineEdge createLineEdge(std::vector<LineSegment*> segments);
    void updateWhiteColor(std::vector<LineSegment*> lineSegments, uint8_t *img) __attribute__((nonnull));
	void findLineGroups();
    std::vector<LineGroup> &getLineGroups();
	color getColor() const { return white; };
};

}  // namespace htwk

#endif  // __LINE_DETECTOR_H__
