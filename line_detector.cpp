#include "line_detector.h"

#include <easy/profiler.h>
#include <robotoption.h>
#include <stl_ext.h>
#include <visualizer.h>

#include <algorithm>
#include <cstring>

using namespace std;
using namespace NaoControl;

namespace htwk {

inline int compareLineSegments(const LineSegment *a, const LineSegment *b) {
    return a->x < b->x;
}

inline int compareLineEdges(const LineEdge *a, const LineEdge *b) {
    return a->segments.size() > b->segments.size();
}

vector<LineGroup> &LineDetector::getLineGroups() {
    return linesList;
}

/**
 * scans image for lines (straight groups of line segments from the RegionClassifier)
 */
void LineDetector::proceed(uint8_t *img, vector<LineSegment *> lineSegments, int q) {
    Timer t("LineDetector", 50);
    EASY_FUNCTION(profiler::colors::Lime100);
    vector<LineSegment *> lineSegmentsSrc = lineSegments;
    if (!linesTmp.empty()) {
        for (LineEdge *ls : linesTmp) {
            delete (ls);
        }
        linesTmp.clear();
    }

    // sort and link lineEdges for faster neighbor-search
    sort(lineSegments.begin(), lineSegments.end(), compareLineSegments);
    for (LineSegment *ls : lineSegments) {
        ls->pred = nullptr;
    }
    LineSegment *pred = nullptr;
    for (LineSegment *ls : lineSegments) {
        ls->minError = numeric_limits<float>::infinity();
        if (pred != nullptr) {
            ls->pred = pred;
        }
        pred = ls;
    }

    // search for near line-edges with similar angle and save the best match for every line-edge
    int rMax = (int)(q * 2.8f);
    int rMin = (int)(q * 0.5f);
    for (LineSegment *ls : lineSegments) {
        LineSegment *neighbor = ls->pred;
        int minX = ls->x - rMax;
        while (neighbor != nullptr && neighbor->x >= minX) {
            int diffY = neighbor->y - ls->y;
            if (diffY > -rMax && diffY < rMax) {
                int diffX = neighbor->x - ls->x;
                int dist = diffX * diffX + diffY * diffY;
                if (dist < rMax * rMax && dist > rMin * rMin) {
                    float d = getError(ls, neighbor);
                    if (d < maxError) {
                        if (d < ls->minError) {
                            ls->minError = d;
                            ls->bestNeighbor = neighbor;
                        }
                        if (d < neighbor->minError) {
                            neighbor->minError = d;
                            neighbor->bestNeighbor = ls;
                        }
                    }
                }
            }
            neighbor = neighbor->pred;
        }
    }

    // search for near line-edges with similar angle and group/link them together into individual neighborhood lists for
    // every line-edge
    rMax = (int)(q * 8);
    rMin = (int)(q * 0.9f);
    for (LineSegment *ls : lineSegments) {
        LineSegment *neighbor = ls->pred;
        int minX = ls->x - rMax;
        while (neighbor != nullptr && neighbor->x >= minX) {
            int diffY = neighbor->y - ls->y;
            if (diffY > -rMax && diffY < rMax) {
                int diffX = neighbor->x - ls->x;
                int dist = diffX * diffX + diffY * diffY;
                if (dist < rMax * rMax && dist > rMin * rMin) {
                    float d = getError2(ls, neighbor);
                    if (d <= 0.75f) {
                        ls->neighbors.push_back(neighbor);
                        neighbor->neighbors.push_back(ls);
                    }
                }
            }
            neighbor = neighbor->pred;
        }
    }
    //---------------------------------------

    // search points, which only have neighbors in one direction (possible end-points from long lines in the image)
    int angles[36];
    vector<LineSegment *> endPoints;
    for (LineSegment *ls : lineSegments) {
        ls->id = 0;
        if (ls->neighbors.size() < 3)
            continue;
        memset(angles, 0, sizeof(int) * 36);
        for (LineSegment *n : ls->neighbors) {
            float dx = ls->x - n->x;
            float dy = ls->y - n->y;
            float angle = atan2(dy, dx);
            int angleIdx = (int)(35.99f * (angle + M_PI) / (M_PI * 2));
            angles[angleIdx]++;
        }
        int maxCnt = 0;
        int cnt = 0;
        for (int i = 0; i < 36 + 36 / 2; i++) {
            if (angles[i % 36] == 0) {
                cnt++;
                if (cnt > maxCnt) {
                    maxCnt = cnt;
                }
            } else {
                cnt = 0;
            }
        }
        if (maxCnt > 18) {
            endPoints.push_back(ls);
        }
    }

    // for every end-point, try to find a line with as much as possible line-edges on it by traveling step by step
    // through the connected neighborhood
    int id = 0;
    lineSegments.clear();
    for (LineSegment *ls : endPoints) {
        if (ls->id > 0)
            continue;
        id++;
        vector<LineSegment *> nStack;
        int sumX = 0;
        int sumY = 0;
//        int cnt = 0;
        for (LineSegment *n : ls->neighbors) {
            int dx = n->x - ls->x;
            int dy = n->y - ls->y;
            if (dx < 0 || (dx == 0 && dy < 0)) {
                dx = -dx;
                dy = -dy;
            }
            sumX += dx;
            sumY += dy;
//            cnt++;
        }
        ls->id = id;
        lineSegments.push_back(ls);
        nStack.push_back(ls);
        while (!nStack.empty()) {
            LineSegment *next = nStack.back();
            nStack.pop_back();
            float len = sqrtf(sumX * sumX + sumY * sumY);
            if (len == 0)
                continue;
            float vx = sumX / len;
            float vy = sumY / len;
            float nx = -vy;
            float ny = vx;
            float d = ls->x * nx + ls->y * ny;
            for (LineSegment *n : next->neighbors) {
                float dist = n->x * nx + n->y * ny - d;
                if (abs(dist) < 4 && n->id == 0) {
                    n->id = id;
                    lineSegments.push_back(n);
                    int dx = n->x - next->x;
                    int dy = n->y - next->y;
                    if (dx < 0 || (dx == 0 && dy < 0)) {
                        dx = -dx;
                        dy = -dy;
                    }
                    sumX += dx;
                    sumY += dy;
                    nStack.push_back(n);
                }
            }
        }
    }
    id++;

    // create lines from line-edges (linear regression)
    int numLinesTmp = id - 1;
    linesTmp.resize(numLinesTmp);
    for (int i = 0; i < id - 1; i++) {
        linesTmp[i] = new LineEdge(i + 1);
    }

    for (LineSegment *ls : lineSegments) {
        ls->parentLine = linesTmp[ls->id - 1];
    }

    updateWhiteColor(lineSegments, img);

    // determine the number of lines found yet
    uint32_t lineCnt = 0;
    for (int i = 0; i < id - 1; i++) {
        if (linesTmp[i]->segments.size() >= minSegmentCnt) {
            lineCnt++;
        }
    }

    // save all detected lines into the destination array
    lineEdges.resize(lineCnt);
    int idn = 0;
    for (int i = 0; i < id - 1; i++) {
        if (linesTmp[i]->segments.size() < minSegmentCnt) {
            continue;
        }
        lineEdges[idn] = linesTmp[i];
        idn++;
    }

    for (LineEdge *ls : lineEdges) {
        ls->update();
    }

    // check for straightness (otherwise its not a line, its maybe a part from an ellipse)
    for (LineEdge *line : lineEdges) {
        line->straight = false;
        if (isStraight(line)) {
            line->straight = true;
        }
    }
    findLineGroups();

    // find line crossings
    crossings.clear();

    float minSize = 4;
    for (const LineGroup &lg : linesList) {
        const LineEdge &lsA = lg.lines[0];
        const LineEdge &lsB = lg.lines[1];
        vector<LineSegment *> left1;
        vector<LineSegment *> left2;
        vector<LineSegment *> right1;
        vector<LineSegment *> right2;
        float maxDist = (lsA.estimateLineWidth() + lsB.estimateLineWidth()) * 2;
        for (LineSegment *ls : lineSegmentsSrc) {

            //            if (!ls->parentLine)
            //                continue;
            //            if (ls->parentLine->segments.size() < minSize)
            //                continue;

            float dx = ls->x - ls->link->x;
            float dy = ls->y - ls->link->y;
            float dist = dx * dx + dy * dy;
            if (dist < 7 * 7)
                continue;
            //
            float side1 = ls->x * lsA.nx + ls->y * lsA.ny - lsA.d;
            if (fabsf(side1) < maxDist) {
                float side2 = ls->x * lsB.nx + ls->y * lsB.ny - lsB.d;
                if (fabsf(side2) < maxDist) {
                    float corr1 = ls->vx * lsA.nx + ls->vy * lsA.ny;
                    float corrOrth1 = ls->vy * lsA.nx - ls->vx * lsA.ny;
                    if (fabsf(corr1) < 0.80f) {
                        if (side1 > 0 && side2 < 0) {
                            if (corrOrth1 > 0) {
                                left1.push_back(ls);
                            } else {
                                left2.push_back(ls);
                            }
                        }
                        if (side1 < 0 && side2 > 0) {
                            if (corrOrth1 > 0) {
                                right1.push_back(ls);
                            } else {
                                right2.push_back(ls);
                            }
                        }
                    }
                }
            }
        }
        if (left1.size() < minSize)
            continue;
        if (left2.size() < minSize)
            continue;
        if (right1.size() < minSize)
            continue;
        if (right2.size() < minSize)
            continue;
        LineEdge leLeft1 = createLineEdge(left1);
        LineEdge leLeft2 = createLineEdge(left2);
        LineEdge leRight1 = createLineEdge(right1);
        LineEdge leRight2 = createLineEdge(right2);

        float angle1 = leLeft1.nx * leRight1.ny - leLeft1.ny * leRight1.nx;
        float angle2 = leLeft2.nx * leRight2.ny - leLeft2.ny * leRight2.nx;
        if (fabsf(angle1) > 0.05f && fabsf(angle2) > 0.05f && angle1 * angle2 > 0) {
            float nx = (leLeft1.nx + leRight1.nx - leLeft2.nx - leRight2.nx) * 0.25f;
            float ny = (leLeft1.ny + leRight1.ny - leLeft2.ny - leRight2.ny) * 0.25f;
            if (angle1 < 0) {
                nx *= -1;
                ny *= -1;
            }
            float xm = (leLeft1.x + leRight1.x + leLeft2.x + leRight2.x) * 0.25f;
            float ym = (leLeft1.y + leRight1.y + leLeft2.y + leRight2.y) * 0.25f;

            auto p1 = getIntersection(lsA.x, lsA.y, -lsA.ny, lsA.nx, xm, ym, -ny, nx);
            if (!p1)
                continue;
            auto p2 = getIntersection(lsB.x, lsB.y, -lsB.ny, lsB.nx, xm, ym, -ny, nx);
            if (!p2)
                continue;
            float mx = (p1->x + p2->x) * 0.5f;
            float my = (p1->y + p2->y) * 0.5f;
            float lineNx = lsA.ny - lsB.ny;
            float lineNy = lsB.nx - lsA.nx;
            if (lineNx * nx + lineNy * ny < 0) {
                lineNx *= -1;
                lineNy *= -1;
            }
            crossings.emplace_back(point_2d{mx, my}, point_2d{lineNx * maxDist, lineNy * maxDist});
        }

        if (!crossings.empty()) {
            detectedLineCrossings += crossings.size();
            if (config.activate_visualization) {
                VisTransPtr ptr = Visualizer::instance().startTransaction({}, "LineDetector", RELATIVE_BODY, REPLACE);
                ptr->addParameter(Parameter::createInt("Crossings", detectedLineCrossings));
                Visualizer::instance().commit(ptr);
            }
        }
    }
}

LineEdge LineDetector::createLineEdge(vector<LineSegment *> segments) {
    float avgNx = 0;
    float avgNy = 0;
    float avgXM = 0;
    float avgYM = 0;
    for (LineSegment *ls : segments) {
        avgNx += ls->vx;
        avgNy += ls->vy;
        avgXM += ls->x;
        avgYM += ls->y;
    }
    avgNx /= segments.size();
    avgNy /= segments.size();
    avgXM /= segments.size();
    avgYM /= segments.size();
    float len = sqrtf(avgNx * avgNx + avgNy * avgNy);
    if (len > 0) {
        avgNx /= len;
        avgNy /= len;
    }
    float d = avgXM * avgNx + avgYM * avgNy;

    float avgNx2 = 0;
    float avgNy2 = 0;
    float avgXM2 = 0;
    float avgYM2 = 0;
    int cnt = 0;
    for (LineSegment *ls : segments) {
        float dist = fabsf(ls->x * avgNx + ls->y * avgNy - d);
        if (dist < 100) {
            avgNx2 += ls->vx;
            avgNy2 += ls->vy;
            avgXM2 += ls->x;
            avgYM2 += ls->y;
            cnt++;
        }
    }
    /* Count could be 0 here -> found by Undefined Sanitizer */
    if (cnt != 0) {
        avgNx2 /= cnt;
        avgNy2 /= cnt;
        avgXM2 /= cnt;
        avgYM2 /= cnt;
    }
    float len2 = sqrtf(avgNx2 * avgNx2 + avgNy2 * avgNy2);
    if (len2 > 0) {
        avgNx2 /= len2;
        avgNy2 /= len2;
    }
    LineEdge result;
    result.x = avgXM2;
    result.y = avgYM2;
    result.nx = avgNx2;
    result.ny = avgNy2;
    return result;
}

optional<point_2d> LineDetector::getIntersection(float px1, float py1, float vx1, float vy1, float px2, float py2,
                                                 float vx2, float vy2) {
    float px = px1;
    float py = py1;
    float qx = px2;
    float qy = py2;

    float ax = vx1;
    float ay = vy1;
    float bx = vx2;
    float by = vy2;

    float ax_ = -ay;
    float ay_ = ax;
    float bx_ = -by;
    float by_ = bx;

    float ab = ax * bx_ + ay * by_;
    if (ab == 0) {
        return {};
    }
    float qb_ = qx * bx_ + qy * by_;
    float pa_ = px * ax_ + py * ay_;
    float sx = 1.f / ab * (qb_ * ax - pa_ * bx);
    float sy = 1.f / ab * (qb_ * ay - pa_ * by);
    return point_2d(sx, sy);
}

void LineDetector::updateWhiteColor(vector<LineSegment *> lineSegments, uint8_t *img) {
    int lineRegionsCnt = 0;
    int whiteCyTmp = 0;
    int whiteCbTmp = 0;
    int whiteCrTmp = 0;
    for (LineSegment *ls : lineSegments) {
        if (ls->parentLine && ls->link) {
            ls->parentLine->segments.push_back(ls);
            int px = (ls->x + ls->link->x) >> 1;
            int py = (ls->y + ls->link->y) >> 1;
            whiteCyTmp += getY(img, px, py);
            whiteCbTmp += getCb(img, px, py);
            whiteCrTmp += getCr(img, px, py);
            lineRegionsCnt++;
        }
    }
    if (lineRegionsCnt > 8) {
        white.cy = whiteCyTmp / lineRegionsCnt;
        white.cb = whiteCbTmp / lineRegionsCnt;
        white.cr = whiteCrTmp / lineRegionsCnt;
    }
}

// finds upper and lower edge for every field line and group them together
void LineDetector::findLineGroups() {
    sort(lineEdges.begin(), lineEdges.end());
    for (LineEdge *line : lineEdges) {
        line->id = -1;
    }

    linesList.clear();
    int minConnectionCnt = 3;
    int id = 1;
    for (LineEdge *le : lineEdges) {
        if (!le->straight || !le->valid)
            continue;
        if (le->id == -1 || (le->id < id && le->matchCnt < minConnectionCnt)) {
            le->id = id;
            id++;
            int maxVal = minConnectionCnt;
            LineEdge *bestNeighbor = nullptr;
            for (LineSegment *ls : le->segments) {
                if (!ls->link)
                    continue;
                LineEdge *neighbor = ls->link->parentLine;
                if (!neighbor)
                    continue;
                if (!neighbor->straight)
                    continue;
                if (neighbor == le)
                    continue;
                if (neighbor->id == id) {
                    neighbor->matchCnt++;
                    if (neighbor->matchCnt >= maxVal) {
                        bestNeighbor = neighbor;
                        maxVal = neighbor->matchCnt;
                    }
                }
                if (neighbor->id == -1 || (neighbor->id < id && neighbor->matchCnt < minConnectionCnt)) {
                    neighbor->id = id;
                    neighbor->matchCnt = 1;
                }
            }
            if (bestNeighbor) {
                int segmentCnt = le->segments.size() + bestNeighbor->segments.size();
                LineGroup lg;
                lg.lines[0] = *le;
                lg.lines[1] = *bestNeighbor;
                lg.points = segmentCnt;
                linesList.push_back(lg);
            }
            id++;
        }
    }
}

LineDetector::LineDetector(const int8_t *lutCb, const int8_t *lutCr, HtwkVisionConfig &config)
    : BaseDetector(lutCb, lutCr, config) {

    if (config.activate_visualization) {
        std::string name = std::string("HTWK/Vision/LineDetector/") + (config.isUpperCam ? "Upper" : "Lower");
        auto *options = new OptionSet(name.c_str());
        options->addOption(new NaoControl::IntOption("minSegmentCnt", &minSegmentCnt, 0, 10, 1));
        options->addOption(new NaoControl::FloatOption("maxError", &maxError, 0.f, 3.f, .05f));
        options->addOption(new NaoControl::FloatOption("isStraightThreshold", &isStraightThreshold, 0.f, 1.f, .001f));
        NaoControl::RobotOption::instance().addOptionSet(options);
    }
}

LineDetector::~LineDetector() {
    if (!linesTmp.empty()) {
        for (LineEdge *ls : linesTmp) {
            delete (ls);
        }
        linesTmp.clear();
    }
}

// test, if a given line is really straight, or maybe curvy
bool LineDetector::isStraight(LineEdge *line) {
    vector<LineSegment *> leftSegments;
    vector<LineSegment *> rightSegments;
    float nx = -line->ny;
    float ny = line->nx;
    float d = line->x * nx + line->y * ny;
    for (LineSegment *ls : line->segments) {
        float dist = ls->x * nx + ls->y * ny - d;
        if (dist < 0) {
            leftSegments.push_back(ls);
        } else {
            rightSegments.push_back(ls);
        }
    }
    LineEdge leftLine(leftSegments);
    LineEdge rightLine(rightSegments);
    float diff = fabs(leftLine.nx * rightLine.nx + leftLine.ny * rightLine.ny);
    return diff > isStraightThreshold;
}

// how similar two line-edges are
float LineDetector::getError(LineSegment *le1, LineSegment *le2) {
    float diffC = le1->vx * le2->vx + le1->vy * le2->vy;
    if (diffC <= 0)
        return numeric_limits<float>::infinity();
    int vx = le1->x - le2->x;
    int vy = le1->y - le2->y;
    float r = invSqrt(vx * vx + vy * vy);
    float err1 = (vx * le1->vx + vy * le1->vy) * r;
    float err2 = (vx * le2->vx + vy * le2->vy) * r;
    return fabsf(err1) + fabsf(err2);
}

// alternative way to determine how similar two line-edges are
float LineDetector::getError2(LineSegment *le1, LineSegment *le2) {
    if (le1->bestNeighbor == nullptr || le2->bestNeighbor == nullptr)
        return numeric_limits<float>::infinity();
    float diffC = le1->vx * le2->vx + le1->vy * le2->vy;
    if (diffC < 0)
        return numeric_limits<float>::infinity();
    float vy1 = le1->x - le1->bestNeighbor->x;
    float vx1 = -(le1->y - le1->bestNeighbor->y);
    float vy2 = le2->x - le2->bestNeighbor->x;
    float vx2 = -(le2->y - le2->bestNeighbor->y);
    int nx = le1->x - le2->x;
    int ny = le1->y - le2->y;
    float r = 1.f / (16 + sqrtf(nx * nx + ny * ny));
    float err1 = (nx * vx1 + ny * vy1) * r;
    float err2 = (nx * vx2 + ny * vy2) * r;
    return max(fabsf(err1), fabsf(err2));
}

}  // namespace htwk
