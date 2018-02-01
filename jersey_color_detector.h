#ifndef __JERSEYCOLORDETECTOR_H__
#define __JERSEYCOLORDETECTOR_H__

#include <cstdint>

#include <vector>

#include "base_detector.h"
#include "range_check.h"
#include "robotrect.h"
#include "team_membership.h"

namespace htwk {

class JerseyColorDetector : protected BaseDetector {
private:
    struct ycbcr32_t{
        int32_t y;
        int32_t cb;
        int32_t cr;
    };

    float offsetColorAngle;
    float offsetColors;
    float sensitivityColors;
    float sensitivityColorAngle;
    float colorAngleBlue;
    float colorAngleRed;
    float pixelCalibrationRatio;
    float pixelSelectionRatio;
    float topRegionHeight;
    float bottomRegionHeight;
    int gridSize;

	int histCbTop[256];
	int histCrTop[256];
	int histYBottom[256];

    TeamMembership determineTeamColor(const uint8_t * const img, RobotRect r) const;

public:
    JerseyColorDetector(int _width, int _height, int8_t *lutCb, int8_t *lutCr) __attribute__((nonnull));
    ~JerseyColorDetector() {}
    float isBlueJersey(const uint8_t * const img, RobotRect rect) __attribute__((nonnull));
	void setHistogramY(const uint8_t * const img, RobotRect rect, int* hist) __attribute__((nonnull));
	void setHistogramCb(const uint8_t * const img, RobotRect rect, int* hist) __attribute__((nonnull));
	void setHistogramCr(const uint8_t * const img, RobotRect rect, int* hist) __attribute__((nonnull));
	int getThreshold(const int* const hist, float ratio) __attribute__((nonnull));
	int getAvgMaxValue(const uint8_t * const img, RobotRect rect, int thres, int ay, int ab, int ar, int by, int bb, int br) __attribute__((nonnull));
};

}
#endif
