#include "field_detector.h"

#include <cmath>
#include <cstring>

using namespace std;

namespace htwk {

FieldDetector::FieldDetector(int width, int height, int8_t *lutCb, int8_t *lutCr, const HtwkVisionConfig &config)
    : BaseDetector(width, height, lutCb, lutCr)
{
    fieldBorderFull = new int[width];

    if(config.isUpperCam) {
        for (int x = 0; x < width; x++) {
            fieldBorderFull[x] = height - 2;
        }
    } else {
        // 0 means a field border on the top of the
        // image, so that all pixels below are valid
        // field pixels
        memset(fieldBorderFull, 0, width*sizeof(*fieldBorderFull));
    }
}

FieldDetector::~FieldDetector() {
    delete [] fieldBorderFull;
}

/**
 * decides which pixels belonging to the playing-field by modelling the
 * fieldborder with up to two lines
 */
void FieldDetector::proceed(
        const uint8_t *const img, const FieldColorDetector *const field,
        const RegionClassifier *const regionClassifier, const bool isUpper) {

	if(!isUpper){
		return;
	}

    borderPoints.clear();
    vector<point_2d> pointsLeft;
    vector<point_2d> pointsDone;

    // search possible points on the field-border
    //(points on edges, with a green color on the bottom and not a green or white
    //color on the top)
    int lineSpacing = regionClassifier->getLineSpacing();
    int offset = lineSpacing / 2;

    for (int x = offset; x < width; x += lineSpacing) {
        Scanline sl = regionClassifier->getScanVertical()[x / lineSpacing];
        for (int i = 0; i < sl.edgeCnt - 1 && i < maxEdgesPerScanline; i++) {
            int px1 = sl.edgesX[i];
            int py1 = sl.edgesY[i];
            int px2 = sl.edgesX[i + 1];
            int py2 = sl.edgesY[i + 1];
            if (i > 0 && !sl.regionsIsGreen[i] && !sl.regionsIsWhite[i] && sl.regionsIsGreen[i - 1]) {
                borderPoints.emplace_back(px1, py1);
            }
            if (i == sl.edgeCnt - 2 && sl.regionsIsGreen[i]) {
                borderPoints.emplace_back(px2, py2);
            }
        }
    }
    // search for the best line matching for the given border points with RANSAC
    // algorithm
    if (borderPoints.size() >= 6) {
        float bestNx = 0;
        float bestNy = 0;
        float bestD = 0;
        int maxSum = 0;
        int pointsCnt = 0;
        for (int i = 0; i < 200; i++) {
            point_2d p1 = borderPoints[(int)(dist(rng) * borderPoints.size())];
            point_2d p2 = borderPoints[(int)(dist(rng) * borderPoints.size())];
            if (p1 == p2)
                continue;
            if (p1.x > p2.x) {
                point_2d tmp = p1;
                p1 = p2;
                p2 = tmp;
            }
            float dx = p1.x - p2.x;
            float dy = p1.y - p2.y;
            float len = std::sqrt(dx * dx + dy * dy);
            if (len < 16) continue;
            float nx = -dy / len;
            float ny = dx / len;
            float d = nx * p1.x + ny * p1.y;
            int sum = 0;
            int cnt = 0;
            for (const point_2d& p : borderPoints) {
                float dist = nx * p.x + ny * p.y - d;
                if (std::fabs(dist) < 1.5f) {
                    cnt++;
                    sum += 1;
                } else if (dist > 4 && dist < 32) {
                    sum -= 1;
                }
            }
            if (sum > maxSum) {
                maxSum = sum;
                pointsCnt = cnt;
                bestNx = nx;
                bestNy = ny;
                bestD = d;
            }
        }
        // if enough points left, search for the best second line matching for the
        // rest of the border points with RANSAC algorithm
        if (pointsCnt >= 5) {
            pointsLeft.clear();
            pointsDone.clear();
            for (const point_2d &p : borderPoints) {
                float dist = fabsf(bestNx * p.x + bestNy * p.y - bestD);
                if (dist >= 1) {
                    pointsLeft.push_back(p);
                } else {
                    pointsDone.push_back(p);
                }
            }
            float bestNx2 = 0;
            float bestNy2 = 0;
            float bestD2 = 0;
            int pointsCnt2 = 0;
            if (pointsLeft.size() >= 4) {
                int maxSum2 = 0;
                for (int i = 0; i < 200; i++) {
                    point_2d p1 = pointsLeft[(int)(dist(rng) * pointsLeft.size())];
                    point_2d p2 = pointsLeft[(int)(dist(rng) * pointsLeft.size())];
                    if (p1 == p2)
                        continue;
                    if (p1.x > p2.x) {
                        point_2d tmp = p1;
                        p1 = p2;
                        p2 = tmp;
                    }
                    float dx = p1.x - p2.x;
                    float dy = p1.y - p2.y;
                    float len = std::sqrt(dx * dx + dy * dy);
                    if (len < 16) continue;
                    float nx = -dy / len;
                    float ny = dx / len;
                    float d = nx * p1.x + ny * p1.y;
                    int sum = 0;
                    int cnt = 0;
                    for (const point_2d &p : pointsLeft) {
                        float dist = nx * p.x + ny * p.y - d;
                        if (fabsf(dist) < 2) {
                            sum += 2;
                            cnt++;
                        } else if (dist > 3 && dist < 20) {
                            sum -= 1;
                        }
                    }
                    for (const point_2d &p : pointsDone) {
                        float dist = nx * p.x + ny * p.y - d;
                        if (dist > 2) {
                            sum -= 2;
                        }
                    }
                    if (sum > maxSum2) {
                        maxSum2 = sum;
                        pointsCnt2 = cnt;
                        bestNx2 = nx;
                        bestNy2 = ny;
                        bestD2 = d;
                    }
                }
            }

            //interpolate the field-border from one or two lines
            for (int x = 0; x < width; x++) {
                if (bestNy == 0) continue;
                int y1 = (int)((bestD - bestNx * x) / bestNy);
                if (pointsCnt2 >= 4 && bestNy2 != 0) {
                    int y2 = (int)((bestD2 - bestNx2 * x) / bestNy2);
                    if (y2 > y1) y1 = y2;
                }
                if (y1 < 0) y1 = 0;
                if (y1 >= height - 2) y1 = height - 2;
                fieldBorderFull[x] = y1;
            }
        } else {
            // interpolate the field-border with default values (if no field border is
            // visible)
            for (int x = 0; x < width; x++) {
                fieldBorderFull[x] = 0;  // 0 means a field border on the top of the
                // image, so that all pixels below are valid
                // field pixels
            }
        }
    }
    // use fieldBorderFull from last frame.
}

}

