#include <region_classifier.h>

#include <cmath>
#include <cstdlib>

#include <ext_math.h>

using namespace ext_math;
using namespace std;

namespace htwk {

int RegionClassifier::tEdge = 2 * 4;  // minimal edge-intensity

int RegionClassifier::maxEdgesInLine =
    3;  // because of multiple edges in one lineregion

int RegionClassifier::maxLineBorder = 6;  // maximal distance (px) between
                                          // line-border and the green
                                          // neighbor-regions

RegionClassifier::RegionClassifier(int *lutCb, int *lutCr, int width,
                                   int height)
    : width_(width), height_(height), lineRegionsCnt(0) {
  this->lutCb = lutCb;
  this->lutCr = lutCr;
  lineSegments = new vector<LineSegment *>();

  scanVertical = new Scanline[width / lineSpacing];
  scanHorizontal = new Scanline[height / lineSpacing];

  for (int i = 0; i < width / lineSpacing; i++) {
    scanVertical[i].vx = 0;
    scanVertical[i].vy = -2;
    scanVertical[i].edgeCnt = 0;
  }

  for (int i = 0; i < height / lineSpacing; i++) {
    scanHorizontal[i].vx = 2;
    scanHorizontal[i].vy = 0;
    scanHorizontal[i].edgeCnt = 0;
  }
}

RegionClassifier::~RegionClassifier() {}

void RegionClassifier::proceed(uint8_t *img, FieldColorDetector *field) {
  // reset scanlines
  for (int i = 0; i < width_ / lineSpacing; i++) {
    scanVertical[i].edgeCnt = 0;
  }
  for (int i = 0; i < height_ / lineSpacing; i++) {
    scanHorizontal[i].edgeCnt = 0;
  }

  int offset = lineSpacing / 2;
  for (int x = offset; x < width_; x += lineSpacing) {
    Scanline *sl = &scanVertical[x / lineSpacing];

    // add first edge (bottom-image-border)
    addEdge(img, sl, x, height_ - 2, -1, false);

    // find edges on vertical scanlines
    scan(img, x, height_ - 2, field, sl);

    // add last edge (field-border)
    addEdge(img, sl, x, 0, 1, false);

    // get region color-values
    getColorsFromRegions(img, sl, (int)sgn(sl->vx), (int)sgn(sl->vy));

    // classify
    classifyGreenRegions(sl, field);
    classifyWhiteRegions(sl);
  }

  for (int y = offset; y < height_; y += lineSpacing) {
    Scanline *sl = &scanHorizontal[y / lineSpacing];

    // find edges on horizontal scanlines
    if ((y / lineSpacing) % 2 == 0) {
      sl->vx = -2;
      addEdge(img, sl, width_ - 1, y, -1, false);
      scan(img, width_ - 1, y, field, sl);
      addEdge(img, sl, 0, y, 1, false);
    } else {
      addEdge(img, sl, 0, y, -1, false);
      scan(img, 0, y, field, sl);
      addEdge(img, sl, width_ - 1, y, 1, false);
    }

    // get region color-values
    getColorsFromRegions(img, sl, (int)sgn(sl->vx), (int)sgn(sl->vy));

    // classify
    classifyGreenRegions(sl, field);
    classifyWhiteRegions(sl);
  }

  // delete all lineSegments
  for (LineSegment *ls : *lineSegments) {
    delete ls;
  }
  lineSegments->clear();
  addSegments(scanVertical, width_ / lineSpacing, img);
  addSegments(scanHorizontal, height_ / lineSpacing, img);
}

void RegionClassifier::addSegments(Scanline *scanlines, int scanlineCnt,
                                   uint8_t *img) {
  for (int j = 0; j < scanlineCnt; j++) {
    Scanline *sl = &scanlines[j];
    for (int i = 1; i < sl->edgeCnt - 1; i++) {
      bool isWhite = sl->regionsIsWhite[i];
      if (isWhite) {
        int k = i + 1;
        for (; k < sl->edgeCnt - 1; k++) {
          if (!sl->regionsIsWhite[k]) break;
        }
        int lineWidth = max(abs(sl->edgesX[i] - sl->edgesX[k]),
                            abs(sl->edgesY[i] - sl->edgesY[k]));
        point_2d vecLeft =
            getGradientVector(sl->edgesX[i], sl->edgesY[i], lineWidth, img);
        point_2d vecRight =
            getGradientVector(sl->edgesX[k], sl->edgesY[k], lineWidth, img);
        LineSegment *lesLeft =
            new LineSegment(sl->edgesX[i], sl->edgesY[i], vecLeft.x, vecLeft.y);
        lineSegments->push_back(lesLeft);
        LineSegment *lesRight = new LineSegment(sl->edgesX[k], sl->edgesY[k],
                                                vecRight.x, vecRight.y);
        lineSegments->push_back(lesRight);
        lesLeft->link = lesRight;
        lesRight->link = lesLeft;
        i = k;
      }
    }
  }
}

point_2d RegionClassifier::getGradientVector(int x, int y, int lineWidth,
                                             uint8_t *img) {
  float gx = 0;
  float gy = 0;
  if (lineWidth <= 3) {
    gx = -(getY(img, x - 1, y - 1) + 2 * getY(img, x - 1, y) +
           getY(img, x - 1, y + 1)) +
         (getY(img, x + 1, y - 1) + 2 * getY(img, x + 1, y) +
          getY(img, x + 1, y + 1));
    gy = -(getY(img, x - 1, y - 1) + 2 * getY(img, x, y - 1) +
           getY(img, x + 1, y - 1)) +
         (getY(img, x - 1, y + 1) + 2 * getY(img, x, y + 1) +
          getY(img, x + 1, y + 1));
  } else {
    gx = -(getY(img, x - 2, y - 2) + 2 * getY(img, x - 2, y) +
           getY(img, x - 2, y + 2)) +
         (getY(img, x + 2, y - 2) + 2 * getY(img, x + 2, y) +
          getY(img, x + 2, y + 2));
    gy = -(getY(img, x - 2, y - 2) + 2 * getY(img, x, y - 2) +
           getY(img, x + 2, y - 2)) +
         (getY(img, x - 2, y + 2) + 2 * getY(img, x, y + 2) +
          getY(img, x + 2, y + 2));
  }
  float iSqrt = invSqrt(gx * gx + gy * gy);
  gx *= iSqrt;
  gy *= iSqrt;

  if (fabs(gx) < fabs(gy)) {
    int pxL = x - 1;
    int pyL = y;
    for (int vy = -matchRadius; vy <= matchRadius; vy++) {
      int py = y + vy;
      if (py < 0 || py >= height_ - 1) return newPoint2D(gx, gy);
      pattern[vy + matchRadius] = getY(img, x, py);
    }
    int minLeft = 0;
    for (int d = 0; d < searchLen; d++) {
      int min = 100000;
      int minDy = 0;
      for (int dy = -searchRadius; dy <= searchRadius; dy++) {
        int sum = 0;
        for (int vy = -matchRadius; vy <= matchRadius; vy++) {
          int ny = pyL + dy + vy;
          if (ny < 0 || ny >= height_ - 1) continue;
          int diff = pattern[vy + matchRadius] - getY(img, pxL, ny);
          sum += diff * diff;
        }
        if (sum < min) {
          min = sum;
          minDy = dy;
        }
      }
      minLeft += min;
      pyL += minDy;
      pxL--;
      if (pxL < 0) break;
    }

    int pxR = x + 1;
    int pyR = y;
    int minRight = 0;
    for (int d = 0; d < searchLen; d++) {
      int min = 100000;
      int minDy = 0;
      for (int dy = -searchRadius; dy <= searchRadius; dy++) {
        int sum = 0;
        for (int vy = -matchRadius; vy <= matchRadius; vy++) {
          int ny = pyR + dy + vy;
          if (ny < 0 || ny >= height_ - 1) continue;
          int diff = pattern[vy + matchRadius] - getY(img, pxR, ny);
          sum += diff * diff;
        }
        if (sum < min) {
          min = sum;
          minDy = dy;
        }
      }
      minRight += min;
      pyR += minDy;
      pxR++;
      if (pxR > width_ - 1) break;
    }
    float vx = pxL - x;
    float vy = pyL - y;
    if (minRight < minLeft) {
      vx = pxR - x;
      vy = pyR - y;
    }
    float len = sqrt(vx * vx + vy * vy);
    if (len > 0) {
      float lenInv = 1. / len;
      vx *= lenInv;
      vy *= lenInv;
    }
    if (gx * vy - gy * vx > 0) {
      gx = vy;
      gy = -vx;
    } else {
      gx = -vy;
      gy = vx;
    }
    return newPoint2D(gx, gy);
  } else {
    for (int vx = -matchRadius; vx <= matchRadius; vx++) {
      int px = x + vx;
      if (px < 0 || px >= width_) return newPoint2D(gx, gy);
      pattern[vx + matchRadius] = getY(img, px, y);
    }
    int pxT = x;
    int pyT = y - 1;
    int minTop = 0;
    for (int d = 0; d < searchLen; d++) {
      int min = 100000;
      int minDx = 0;
      for (int dx = -searchRadius; dx <= searchRadius; dx++) {
        int sum = 0;
        for (int vx = -matchRadius; vx <= matchRadius; vx++) {
          int nx = pxT + dx + vx;
          if (nx < 0 || nx >= width_ - 1) continue;
          int diff = pattern[vx + matchRadius] - getY(img, nx, pyT);
          sum += diff * diff;
        }
        if (sum < min) {
          min = sum;
          minDx = dx;
        }
      }
      minTop += min;
      pxT += minDx;
      pyT--;
      if (pyT < 0) break;
    }

    int pxB = x;
    int pyB = y + 1;
    int minBottom = 0;
    for (int d = 0; d < searchLen; d++) {
      int min = 100000;
      int minDx = 0;
      for (int dx = -searchRadius; dx <= searchRadius; dx++) {
        int sum = 0;
        for (int vx = -matchRadius; vx <= matchRadius; vx++) {
          int nx = pxB + dx + vx;
          if (nx < 0 || nx >= width_ - 1) continue;
          int diff = pattern[vx + matchRadius] - getY(img, nx, pyB);
          sum += diff * diff;
        }
        if (sum < min) {
          min = sum;
          minDx = dx;
        }
      }
      minBottom += min;
      pxB += minDx;
      pyB++;
      if (pyB >= height_ - 1) break;
    }
    float vx = pxT - x;
    float vy = pyT - y;
    if (minBottom < minTop) {
      vx = pxB - x;
      vy = pyB - y;
    }
    float len = sqrt(vx * vx + vy * vy);
    if (len > 0) {
      float lenInv = 1. / len;
      vx *= lenInv;
      vy *= lenInv;
    }
    if (gx * vy - gy * vx > 0) {
      gx = vy;
      gy = -vx;
    } else {
      gx = -vy;
      gy = vx;
    }
    return newPoint2D(gx, gy);
  }
}

void RegionClassifier::classifyWhiteRegions(Scanline *sl) {
  for (int i = 2; i < sl->edgeCnt - 1; i++) {
    if (sl->regionsIsGreen[i]) {  // upper green region found?

      // find lower green-region
      int lowerIdx;
      for (lowerIdx = i - 2; lowerIdx >= 0; lowerIdx--) {
        if (sl->regionsIsGreen[lowerIdx]) {
          break;
        }
      }
      if (lowerIdx >= 0) {
        // find strongest lower edge
        int maxDiff = 0;
        int maxJ = -1;
        for (int j = lowerIdx + 1; j < i; j++) {
          int diff = sl->regionsCy[j] - sl->regionsCy[j - 1];
          if (diff > maxDiff) {
            maxDiff = diff;
            maxJ = j;
          }
        }
        if (maxJ >= 0) {
          int lowerGap = max(abs(sl->edgesX[lowerIdx + 1] - sl->edgesX[maxJ]),
                             abs(sl->edgesY[lowerIdx + 1] - sl->edgesY[maxJ]));
          if (lowerGap <= maxLineBorder) {
            // find strongest upper edge
            int minDiff = 0;
            int minK = -1;
            for (int k = maxJ + 1; k <= i; k++) {
              int diff = sl->regionsCy[k] - sl->regionsCy[k - 1];
              if (diff < minDiff) {
                minDiff = diff;
                minK = k;
              }
            }
            if (minK >= 0) {
              int upperGap = max(abs(sl->edgesX[minK] - sl->edgesX[i]),
                                 abs(sl->edgesY[minK] - sl->edgesY[i]));
              int lineWidth = max(abs(sl->edgesX[minK] - sl->edgesX[maxJ]),
                                  abs(sl->edgesY[minK] - sl->edgesY[maxJ]));
              bool isOnlyGreen = true;
              if (lineWidth > 10) {
                for (int d = maxJ; d < minK; d++) {
                  if (!sl->regionsIsGreen[d]) {
                    isOnlyGreen = false;
                    break;
                  }
                }
              } else {
                isOnlyGreen = false;
              }
              if (upperGap <= maxLineBorder && !isOnlyGreen) {
                // classify line-regions (from lower to upper edge)
                for (int d = maxJ; d < minK; d++) {
                  sl->regionsIsWhite[d] = true;
                }
              }
            }
          }
        }
      }
    }
  }
}

void RegionClassifier::classifyGreenRegions(Scanline *sl,
                                            FieldColorDetector *field) {
  for (int i = 1; i < sl->edgeCnt - 1; i++) {
    sl->regionsIsGreen[i] = false;
    sl->regionsIsWhite[i] = false;
    if (field->isGreen(sl->regionsCy[i], sl->regionsCb[i], sl->regionsCr[i])) {
      sl->regionsIsGreen[i] = true;
    } else {
      if (field->isGreen(sl->regionsCy[i - 1], sl->regionsCb[i - 1],
                         sl->regionsCr[i - 1])) {
        int dCy = sl->regionsCy[i] - sl->regionsCy[i - 1];
        int dCb = sl->regionsCb[i] - sl->regionsCb[i - 1];
        int dCr = sl->regionsCr[i] - sl->regionsCr[i - 1];
        int dist = dCy * dCy + dCb * dCb + dCr * dCr;
        if (dist < 160) {
          sl->regionsIsGreen[i] = true;
        }
      }
    }
  }

  for (int i = sl->edgeCnt - 2; i >= 0; i--) {
    if (sl->regionsIsGreen[i]) continue;
    if (field->isGreen(sl->regionsCy[i], sl->regionsCb[i], sl->regionsCr[i])) {
      sl->regionsIsGreen[i] = true;
    } else {
      if (field->isGreen(sl->regionsCy[i + 1], sl->regionsCb[i + 1],
                         sl->regionsCr[i + 1])) {
        int dCy = sl->regionsCy[i] - sl->regionsCy[i + 1];
        int dCb = sl->regionsCb[i] - sl->regionsCb[i + 1];
        int dCr = sl->regionsCr[i] - sl->regionsCr[i + 1];
        int dist = dCy * dCy + dCb * dCb + dCr * dCr;
        if (dist < 160) {
          sl->regionsIsGreen[i] = true;
        }
      }
    }
  }
}

// estimate region-colors (median of 5 yCbCr-pixel-values)

void RegionClassifier::getColorsFromRegions(uint8_t *img, Scanline *sl,
                                            int dirX, int dirY) const {
  int dataCy[5];
  int dataCb[5];
  int dataCr[5];
  for (int i = 0; i < sl->edgeCnt - 1; i++) {
    int px1 = sl->edgesX[i];
    int py1 = sl->edgesY[i];
    int px2 = sl->edgesX[i + 1];
    int py2 = sl->edgesY[i + 1];
    int len = max(abs(px1 - px2), abs(py1 - py2)) - 1;
    int step = max(1, len / 5);  // must be DIV!
    int vx = dirX * step;
    int vy = dirY * step;
    px1 += vx / 2 + dirX;
    py1 += vy / 2 + dirY;
    int cnt = 0;
    for (; cnt < 5 && cnt < len; px1 += vx, py1 += vy, cnt++) {
      dataCy[cnt] = getY(img, px1, py1);
      dataCb[cnt] = getCb(img, px1, py1);
      dataCr[cnt] = getCr(img, px1, py1);
    }
    if (cnt == 5) {
      sl->regionsCy[i] =
          medianOfFive(dataCy[0], dataCy[1], dataCy[2], dataCy[3], dataCy[4]);
      sl->regionsCb[i] =
          medianOfFive(dataCb[0], dataCb[1], dataCb[2], dataCb[3], dataCb[4]);
      sl->regionsCr[i] =
          medianOfFive(dataCr[0], dataCr[1], dataCr[2], dataCr[3], dataCr[4]);
    } else if (cnt == 4) {
      sl->regionsCy[i] =
          medianOfThree(dataCy[0], (dataCy[1] + dataCy[2]) >> 1, dataCy[3]);
      sl->regionsCb[i] =
          medianOfThree(dataCb[0], (dataCb[1] + dataCb[2]) >> 1, dataCb[3]);
      sl->regionsCr[i] =
          medianOfThree(dataCr[0], (dataCr[1] + dataCr[2]) >> 1, dataCr[3]);
    } else if (cnt == 3) {
      sl->regionsCy[i] = (dataCy[0] + 6 * dataCy[1] + dataCy[2]) >> 3;
      sl->regionsCb[i] = (dataCb[0] + 6 * dataCb[1] + dataCb[2]) >> 3;
      sl->regionsCr[i] = (dataCr[0] + 6 * dataCr[1] + dataCr[2]) >> 3;
    } else if (cnt == 2) {
      sl->regionsCy[i] = (dataCy[0] + dataCy[1]) >> 1;
      sl->regionsCb[i] = (dataCb[0] + dataCb[1]) >> 1;
      sl->regionsCr[i] = (dataCr[0] + dataCr[1]) >> 1;
    } else if (cnt == 1) {
      sl->regionsCy[i] = dataCy[0];
      sl->regionsCb[i] = dataCb[0];
      sl->regionsCr[i] = dataCr[0];
    } else if (cnt == 0) {
      sl->regionsCy[i] = getY(img, px2, py2);
      sl->regionsCb[i] = getCb(img, px2, py2);
      sl->regionsCr[i] = getCr(img, px2, py2);
    }
  }
}

// search edges along scanline

void RegionClassifier::scan(uint8_t *img, int xPos, int yPos,
                            FieldColorDetector *field,
                            Scanline *scanline) const {
  int vecX = scanline->vx;
  int vecY = scanline->vy;
  int lastCy = getY(img, xPos, yPos);
  int lastCb = getCb(img, xPos, yPos);
  int lastCr = getCr(img, xPos, yPos);
  int gMax = -tEdge;
  int gMin = tEdge;
  int xPeak = xPos;
  int yPeak = yPos;
  xPos += vecX;
  yPos += vecY;
  bool wasGreen = field->isGreen(lastCy, lastCb, lastCr);
  while (xPos >= 0 && xPos < width_ && yPos >= 0 && yPos < height_ - 1) {
    int cy = getY(img, xPos, yPos);
    int cb = getCb(img, xPos, yPos);
    int cr = getCr(img, xPos, yPos);
    int g = cy - lastCy;
    bool isGreen = field->isGreen(cy, cb, cr);
    bool greenEdge = wasGreen && !isGreen;
    if (greenEdge) {
      g = +tEdge + 1;
    }
    if (g > gMax) {
      if (gMin < -tEdge) {
        if (!addEdge(img, scanline, xPeak, yPeak, gMin, true)) break;
      }
      gMax = g;
      gMin = tEdge;
      xPeak = xPos - vecX / 2;
      yPeak = yPos - vecY / 2;
    }
    if (g < gMin) {
      if (gMax > tEdge) {
        if (!addEdge(img, scanline, xPeak, yPeak, gMax, true)) break;
      }
      gMin = g;
      gMax = -tEdge;
      xPeak = xPos - vecX / 2;
      yPeak = yPos - vecY / 2;
    }

    if (isGreen != wasGreen) {
      wasGreen = isGreen;
    }
    lastCy = cy;
    xPos += vecX;
    yPos += vecY;
  }
}

// add edge-position and magnitude to scanline
// returns false, when current edges-count per scanline is higher than
// "maxEdgesPerScanline"

bool RegionClassifier::addEdge(uint8_t *img, Scanline *scanline, int xPeak,
                               int yPeak, int edgeIntensity,
                               bool optimize) const {
  int xBest = xPeak;
  int yBest = yPeak;
  int maxVec = max(abs(scanline->vx), abs(scanline->vy));

  // optimize edge-position to 1px accuracy if needed (with local scan)
  if (optimize && maxVec > 1) {
    int vx = scanline->vx / maxVec;
    int vy = scanline->vy / maxVec;
    if (xPeak < 2 || xPeak >= width_ - 2 || yPeak < 2 || yPeak >= height_ - 3)
      return true;
    xPeak -= vx * 2;
    yPeak -= vy * 2;
    int lastCy2 = getY(img, xPeak, yPeak);
    ;
    xPeak += vx;
    yPeak += vy;
    int lastCy = getY(img, xPeak, yPeak);
    ;
    int max = 0;
    for (int i = 0; i < 3; i++) {
      xPeak += vx;
      yPeak += vy;
      int cy = getY(img, xPeak, yPeak);
      int g = (cy - lastCy2);
      int gSign = g * edgeIntensity;
      if (gSign > max) {
        max = gSign;
        xBest = xPeak;
        yBest = yPeak;
      }
      lastCy2 = lastCy;
      lastCy = cy;
    }
    xBest -= vx;
    yBest -= vy;
  }

  // save edge-position with highest magnitude to scanline:
  int edgeCnt = scanline->edgeCnt;
  if (edgeCnt < maxEdgesPerScanline) {
    scanline->edgesX[edgeCnt] = xBest;
    scanline->edgesY[edgeCnt] = yBest;
    scanline->edgesIntensity[edgeCnt] = edgeIntensity;
    scanline->link[edgeCnt] = 0;
    scanline->regionsIsGreen[edgeCnt] = false;
    scanline->regionsIsWhite[edgeCnt] = false;
    scanline->regionsCy[edgeCnt] = 0;
    scanline->regionsCb[edgeCnt] = 0;
    scanline->regionsCr[edgeCnt] = 0;
    scanline->edgeCnt++;
    return true;
  } else {
    return false;
  }
}

vector<LineSegment *> RegionClassifier::getLineSegments(
    const int *const fieldborder) {
  vector<LineSegment *> segmentsOnField;
  for (LineSegment *ls : *lineSegments) {
    int py = (ls->y + ls->link->y) / 2;
    if (fieldborder[ls->x] <= py + 6) {
      segmentsOnField.push_back(ls);
    }
  }
  return segmentsOnField;
}

}  // namespace htwk
