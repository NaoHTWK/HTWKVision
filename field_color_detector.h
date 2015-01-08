#ifndef __FIELD_COLOR_DETECTOR_H__
#define __FIELD_COLOR_DETECTOR_H__

#include <stdint.h>
#include <vector>

#include <color.h>
#include <range_check.h>

#define HIST_SIZE 25

namespace htwk {

class FieldColorDetector{
private:
	int histY[256];
	int histCb[256];
	int histCr[256];
	int lut[HIST_SIZE*HIST_SIZE*HIST_SIZE];
	int histYCbCr[HIST_SIZE*HIST_SIZE*HIST_SIZE];
	int histYCbCrSmooth[HIST_SIZE*HIST_SIZE*HIST_SIZE];
	int histYCbCrSmooth2[HIST_SIZE*HIST_SIZE*HIST_SIZE];

	static const int pixelSpacing;
	static const int minFieldArea;
	static const int colorBorder;

	int greenCr;
	int greenCb;
	int greenCy;
	int seedCr;
	int seedCb;
	int seedY;

	const int width;
	const int height;
	const int *lutCb;
	const int *lutCr;

public:
	inline uint8_t getY(const uint8_t * const img,int32_t x,int32_t y) const __attribute__((nonnull)) __attribute__((pure)){
		CHECK_RANGE(x,0,width-1);
		CHECK_RANGE(y,0,height-1);
		return img[(x+y*width)<<1];
	}
	inline uint8_t getCb(const uint8_t * const img,int32_t x,int32_t y) const __attribute__((nonnull)) __attribute__((pure)){
		CHECK_RANGE(x,0,width-1);
		CHECK_RANGE(y,0,height-1);
		return img[((x+y*width)<<1)+lutCb[x]];
	}
	inline uint8_t getCr(const uint8_t * const img,int32_t x,int32_t y) const __attribute__((nonnull)) __attribute__((pure)){
		CHECK_RANGE(x,0,width-1);
		CHECK_RANGE(y,0,height-1);
		return img[((x+y*width)<<1)+lutCr[x]];
	}

    inline void setY(uint8_t* const img, const int32_t x,int32_t y, const uint8_t c) __attribute__((nonnull)){
        CHECK_RANGE(x,0,width-1);
        CHECK_RANGE(y,0,height-1);
        img[(x+y*width)<<1]=c;
    }

    FieldColorDetector(int width, int height, int *lutCb, int *lutCr) __attribute__((nonnull));
	~FieldColorDetector();

	void proceed(const uint8_t * const img) __attribute__((nonnull));
	void searchInitialSeed(const uint8_t * const img);
	void smoothHist(const int* const histSrc, int* histDest);
	void createLUT(const int* const hist);
	void smoothLUT(std::vector<color> &entries, int r);
	static int getStableMin(const int* const hist, int thres);
	static int getPeak(const int* const hist);
	bool isGreen(int cy, int cb, int cr) const;
	color getColor() const;
	void resetArrays();
};

} // namespace htwk

#endif  // __FIELD_COLOR_DETECTOR_H__
