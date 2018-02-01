#ifndef __REGION_CLASSIFIER_H__
#define __REGION_CLASSIFIER_H__

#include <cstdint>
#include <vector>

#include "base_detector.h"
#include "field_color_detector.h"
#include "linesegment.h"
#include "point_2d.h"

namespace htwk {

#define maxEdgesPerScanline 16 //to reduce memory amount and cpu time

struct Scanline{
	int vx,vy;
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

class RegionClassifier : protected BaseDetector
{
private:
    RegionClassifier() = delete;
	RegionClassifier(const RegionClassifier & cpy) = delete;
	RegionClassifier operator= (RegionClassifier & cpy) = delete;

    static void classifyGreenRegions(Scanline *sl, FieldColorDetector *field) __attribute__((nonnull));
    static void classifyWhiteRegions(Scanline *sl) __attribute__((nonnull));
    bool addEdge(uint8_t *img, Scanline *scanline, int xPeak, int yPeak, int edgeIntensity, bool optimize) const __attribute__((nonnull));

    void scan(uint8_t *img, int xPos, int yPos, FieldColorDetector *field, Scanline *scanline) const __attribute__((nonnull));
    point_2d getGradientVector(int x, int y, int lineWidth, uint8_t *img) __attribute__((nonnull));
    void getColorsFromRegions(uint8_t *img, Scanline *sl, int dirX, int dirY) const __attribute__((nonnull));
    void addSegments(Scanline *scanlines, int scanlineCnt, uint8_t *img)  __attribute__((nonnull));

    Scanline *scanVertical;
    Scanline *scanHorizontal;

    static int tEdge;
    static int maxEdgesInLine;
    static int maxLineBorder;
    int lineRegionsCnt;
    static const int matchRadius=2;
    int pattern[matchRadius*2+1];

public:
    const int lineSpacing;
    static const int searchRadius=2;
    static const int searchLen=8;

	std::vector<LineSegment*> *lineSegments;

    RegionClassifier(int width, int height, bool isUpperCam, int8_t *lutCb, int8_t *lutCr) __attribute__((nonnull));
	~RegionClassifier();

    void proceed(uint8_t *img, FieldColorDetector *field) __attribute__((nonnull));
    int getScanVerticalSize() { return width/lineSpacing; }
    int getScanHorizontalSize() { return height/lineSpacing; }
    std::vector<LineSegment*> getLineSegments(const int* const fieldborder);
    int getLineSpacing() const { return lineSpacing; }
    Scanline *getScanVertical() const { return scanVertical; }
    Scanline *getScanHorizontal() const { return scanHorizontal; }
};

}  // namespace htwk

#endif  // __REGION_CLASSIFIER_H__
