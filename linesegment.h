#ifndef LINESEGMENT_H_
#define LINESEGMENT_H_

#include <limits>
#include <vector>

#include "lineedge.h"

namespace htwk {
class LineSegment {
public:
  LineSegment(const LineSegment& l) = delete;
  LineSegment(LineSegment&& l) = delete;
  LineSegment& operator=(const LineSegment&) = delete;
  LineSegment& operator=(LineSegment&&) = delete;

  int x, y;
  float vx, vy;
  int id;  // die ID ist bei Segmenten, die zur gleichen Linienkante gehÃ¶ren,
           // identisch

  // Datenstrukturen, um zusammengehÃ¶rige Liniensegmente zu gruppieren

  std::vector<LineSegment *> neighbors;
  LineSegment *bestNeighbor {nullptr};
  LineSegment *pred {nullptr};
  LineSegment *link {nullptr};
  LineEdge *parentLine {nullptr};
  float minError {std::numeric_limits<float>::max()};

  LineEdge *edge1 {nullptr};
  LineEdge *edge2 {nullptr};

  LineSegment(int x, int y, float vecX, float vecY)
      : x(x), y(y), vx(vecX), vy(vecY), id(0) {}
  ~LineSegment() = default;
};

}  // namespace htwk

#endif /* LINESEGMENT_H_ */
