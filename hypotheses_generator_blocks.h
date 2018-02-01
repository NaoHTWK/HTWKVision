#ifndef HYPOTHESES_GENERATOR_BLOCKS_H
#define HYPOTHESES_GENERATOR_BLOCKS_H

#include <point_2d.h>
#include <base_detector.h>
#include <field_color_detector.h>
#include <hypotheses_generator.h>
#include <integral_image.h>
#include <object_hypothesis.h>

namespace htwk {
class HypothesesGeneratorBlocks : public HypothesesGenerator, protected BaseDetector {
private:
    std::vector<ObjectHypothesis> hypoList;

    static constexpr float SCALE_MEDIUM=1.02;
    IntegralImage* integral;

//    static constexpr int CAM_WIDTH=640;
//    static constexpr int CAM_HEIGHT=480;
//    static constexpr float CAM_FOV=1.03;
//    const float f=CAM_WIDTH/2./tan(CAM_FOV/2);
    float rollRad;
    float pitchRad;

    static constexpr unsigned int MAX_NUM_HYPOTHESES=16;
    const float MIN_OBJECT_RADIUS_NORM=8.16;
    static constexpr float MIN_OBJECT_RADIUS=7;
    static constexpr float borderCompensation=21.49;
    static constexpr int MIN_RATING=239;
    int *ratingImg;

    static constexpr int BLOCK_SIZE=32;
    static constexpr int blockSize = BLOCK_SIZE / IntegralImage::INTEGRAL_SCALE;
    const int numBlockX;
    const int numBlockY;
    std::vector<bool> isBlockUsable;
    int *blockMeanValues;
    int *blockMaxX;
    int *blockMaxY;
    int *blockObjectRadius;

//    static constexpr float BOT_HEIGHT=0.4694;
    static constexpr float BALL_SIZE=37.025;//ballradius in px, wenn 1 Meter entfernt
//    static constexpr float SIZE_OFFSET=-3;//ballradiusoffset in px, wenn 1 Meter entfernt

    void selectBestHypotheses();
    void setBlockValues(uint8_t *img, const int * const border) __attribute__((nonnull));
    void searchHypotheses(float objectScale, const int * const border) __attribute__((nonnull));
    void getRatingGrid(float objectScale, const int * const border) const __attribute__((nonnull));
    void findMaxima(float objectScale,  const int * const border) __attribute__((nonnull));
    bool containsObject(const ObjectHypothesis &hyp) const;
    void getRating(int y, int &maxY, int stepSize, int &maxRating, int r, int r2, int &maxX, int x) const;
    inline int getArea(int px1, int py1, int px2, int py2) const {
        return (px2-px1)*(py2-py1);
    }

public:
    HypothesesGeneratorBlocks(int width, int height, int8_t *lutCb, int8_t *lutCr) __attribute__((nonnull));
    ~HypothesesGeneratorBlocks() override;

    void proceed(uint8_t *img, const int * const fieldborder, float camPitch, float camRoll, IntegralImage *integralImg) override;
    std::vector<ObjectHypothesis> getHypotheses() const override { return hypoList; }
    int* getRatingImg() override {return ratingImg;}
};
}//namespace htwk
#endif // HYPOTHESES_GENERATOR_BLOCKS_H
