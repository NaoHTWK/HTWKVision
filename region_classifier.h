#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "base_detector.h"
#include "field_color_detector.h"
#include "linesegment.h"
#include "point_2d.h"

namespace htwk {

#define maxEdgesPerScanline 24  // to reduce memory amount and cpu time

struct Scanline {
    int vx, vy;
    int edgeCnt;
    int edgesX[maxEdgesPerScanline];
    int edgesY[maxEdgesPerScanline];
    int edgesIntensity[maxEdgesPerScanline];
    int regionsCy[maxEdgesPerScanline];
    int regionsCb[maxEdgesPerScanline];
    int regionsCr[maxEdgesPerScanline];
    bool regionsIsGreen[maxEdgesPerScanline];
    bool regionsIsWhite[maxEdgesPerScanline];
    int link[maxEdgesPerScanline];
};

class RegionClassifier : protected BaseDetector {
private:
    static void classifyGreenRegions(Scanline *sl, FieldColorDetector *field) __attribute__((nonnull));
    static void classifyWhiteRegions(Scanline *sl) __attribute__((nonnull));
    bool addEdge(uint8_t *img, Scanline *scanline, int xPeak, int yPeak, int edgeIntensity, bool optimize) const
            __attribute__((nonnull));

    // TODO: We need a nicer architecture for this so we can unify the 3 scan methods.
    void scan(uint8_t *img, int xPos, int yPos, FieldColorDetector *field, Scanline *scanline) const
            __attribute__((nonnull));
    void scan_avg_x(uint8_t *img, int xPos, int yPos, FieldColorDetector *field, Scanline *scanline) const
            __attribute__((nonnull));
    void scan_avg_y(uint8_t *img, int xPos, int yPos, FieldColorDetector *field, Scanline *scanline) const
            __attribute__((nonnull));
    point_2d getGradientVector(int x, int y, int lineWidth, uint8_t *img) __attribute__((nonnull));
    void getColorsFromRegions(uint8_t *img, Scanline *sl, int dirX, int dirY) const __attribute__((nonnull));
    void addSegments(Scanline *scanlines, int scanlineCnt, uint8_t *img) __attribute__((nonnull));

    Scanline *scanVertical;
    Scanline *scanHorizontal;

    static int tEdge;
    static int maxEdgesInLine;
    static int maxLineBorder;
    static int greenRegionColorDist;
    int lineRegionsCnt;
    static const int matchRadius = 2;
    int pattern[matchRadius * 2 + 1];
    bool upperCam;

public:
    const int lineSpacing;
    static const int searchRadius = 2;
    static const int searchLen = 8;

    std::vector<LineSegment*> lineSegments;

    RegionClassifier(const RegionClassifier &cpy) = delete;
    RegionClassifier(int8_t *lutCb, int8_t *lutCr, HtwkVisionConfig &config) __attribute__((nonnull));
    ~RegionClassifier();
    RegionClassifier(RegionClassifier &) = delete;
    RegionClassifier(RegionClassifier &&) = delete;
    RegionClassifier &operator=(const RegionClassifier &cpy) = delete;
    RegionClassifier &operator=(RegionClassifier &&cpy) = delete;

    void proceed(uint8_t *img, FieldColorDetector *field) __attribute__((nonnull));
    int getScanVerticalSize() {
        return width / lineSpacing;
    }
    int getScanHorizontalSize() {
        return height / lineSpacing;
    }
    std::vector<LineSegment*> getLineSegments(const std::vector<int> &fieldborder);
    Scanline *getScanVertical() const {
        return scanVertical;
    }
    Scanline *getScanHorizontal() const {
        return scanHorizontal;
    }
};

}  // namespace htwk
