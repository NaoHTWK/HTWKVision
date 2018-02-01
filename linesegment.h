#ifndef LINESEGMENT_H_
#define LINESEGMENT_H_

#include <limits>
#include <vector>

#include "lineedge.h"

namespace htwk {
class LineSegment {
public:
  LineSegment(LineSegment &l) = delete;
  void operator=(LineSegment const &) = delete;

  int x, y;
  float vx, vy;
  int id;  // die ID ist bei Segmenten, die zur gleichen Linienkante gehÃ¶ren,
           // identisch

  // Datenstrukturen, um zusammengehÃ¶rige Liniensegmente zu gruppieren

  std::vector<LineSegment *> neighbors;
  LineSegment *bestNeighbor;
  LineSegment *pred;
  LineSegment *link;
  LineEdge *parentLine;
  float minError;

  LineEdge *edge1;
  LineEdge *edge2;

  LineSegment(int x, int y, float vecX, float vecY) {
    this->x = x;
    this->y = y;
    this->vx = vecX;
    this->vy = vecY;
    this->bestNeighbor = nullptr;
    this->parentLine = nullptr;
    this->id = 0;
    this->link = nullptr;
    pred = nullptr;
    minError = std::numeric_limits<float>::max();
    edge1 = edge2 = nullptr;
  }
  ~LineSegment() {}
};

}  // namespace htwk

#endif /* LINESEGMENT_H_ */
