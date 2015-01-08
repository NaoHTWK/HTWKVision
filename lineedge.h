#ifndef LINEEDGE_H
#define LINEEDGE_H

#include <vector>

namespace htwk {
class LineSegment;

class LineEdge {
public:
	std::vector<LineSegment*> segments;
	float px1,py1,px2,py2;
	float nx;
	float ny;
	float d;
	float x;
	float y;
	int id;
	int matchCnt;
	bool straight;
	bool valid;

	LineEdge();
	~LineEdge();
	LineEdge(int id);
	LineEdge(const std::vector<LineSegment*>& segments);
	void update();
	void setVector(float vx, float vy);
	float estimateLineWidth();

};

}  // namespace htwk

#endif // LINEEDGE_H
