#ifndef __FIELD_COLOR_DETECTOR_H__
#define __FIELD_COLOR_DETECTOR_H__

#include <array>
#include <cstdint>
#include <vector>

#include "base_detector.h"
#include "color.h"
#include "range_check.h"

namespace htwk {

class FieldColorDetector : public BaseDetector {
private:
    static constexpr int NUM_FEATURES = 7;

    int minCy2{};
    int maxCy2{};
    int minCb2{};
    int maxCb2{};
    int minCr2{};
    int maxCr2{};

    std::array<int, 256> histY{{0}};
    std::array<int, 256> histCb{{0}};
    std::array<int, 256> histCr{{0}};
    float features[NUM_FEATURES]{};

	static const int pixelSpacing;
	static const int minFieldArea;
	static const int colorBorder;
	static const float greenGain;
	static const float thetas[62];

    int greenCr{};
    int greenCb{};
    int greenCy{};
    int seedCr{};
    int seedCb{};
    int seedY{};

public:
    int minCy{};
    int maxCy{};
    int minCb{};
    int maxCb{};
    int minCr{};
    int maxCr{};

    using BaseDetector::BaseDetector;

    void proceed(const uint8_t* img) __attribute__((nonnull));
    void searchInitialSeed(const uint8_t* img);
	void setYCbCrCube(float* features);
    void extractFeatures(const uint8_t* img, float* features);
    static int getStableMin(const std::array<int, 256>& hist, int thres);
    static int getPeak(const std::array<int, 256>& hist);

	/**
	 * Test, if a given yuv-color matches the field-color (used to detect all pixels on the carpet)
	 */
    inline bool maybeGreen(const color& c) const {
        return maybeGreen(c.cy, c.cb, c.cr);
    }

	inline bool maybeGreen(int cyReal, int cbReal, int crReal) const {
		if(crReal>maxCr2)return false; //44705
		if(cyReal>maxCy2)return false; //30018
		if(cbReal>maxCb2)return false; //7576
		if(cbReal<minCb2)return false; //2171
		if(crReal<minCr2)return false; //353
		if(cyReal<minCy2)return false; //68
		return true;
	}

    /**
     * Test, if a given yuv-color matches the field-color (used to detect all pixels on the carpet)
     */
    inline bool isGreen(int cyReal, int cbReal, int crReal) const {
    	if(crReal>maxCr)return false; //44705
		if(cyReal>maxCy)return false; //30018
		if(cbReal>maxCb)return false; //7576
		if(cbReal<minCb)return false; //2171
		if(crReal<minCr)return false; //353
		if(cyReal<minCy)return false; //68
		return true;
    }

    inline bool isGreen(const uint8_t * const img, int x, int y) const {
		int cr=getCr(img,x,y);
    	int cy=getY(img,x,y);
		int cb=getCb(img,x,y);
        return 1-(1&
                  (((cy-minCy)>>31) | ((maxCy-cy)>>31)|
                   ((cb-minCb)>>31) | ((maxCb-cb)>>31)|
                   ((cr-minCr)>>31) | ((maxCr-cr)>>31)));
//       	if(crReal>maxCr)return false; //44705
//   		if(cyReal>maxCy)return false; //30018
//   		if(cbReal>maxCb)return false; //7576
//   		if(cbReal<minCb)return false; //2171
//   		if(crReal<minCr)return false; //353
//   		if(cyReal<minCy)return false; //68
//   		return true;
       }

    inline bool isGreen(const color& c) const {
    	if(c.cr>maxCr)return false; //44705
		if(c.cy>maxCy)return false; //30018
		if(c.cb>maxCb)return false; //7576
		if(c.cb<minCb)return false; //2171
		if(c.cr<minCr)return false; //353
		if(c.cy<minCy)return false; //68
		return true;
    }

    color getColor() const {
        return {greenCy, greenCb, greenCr};
    }

	void resetArrays();
};

} // namespace htwk

#endif  // __FIELD_COLOR_DETECTOR_H__
