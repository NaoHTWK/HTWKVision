#ifndef HYPOTHESES_GENERATOR_BLUR_H
#define HYPOTHESES_GENERATOR_BLUR_H

#include <base_detector.h>
#include <hypotheses_generator.h>
#include <point_2d.h>

#include <cmath>

namespace htwk {

class HypothesesGeneratorBlur : public HypothesesGenerator, protected BaseDetector {
private:
    static constexpr int RATING_SCALE = 2;  // only 1,2 or 4
    static const float MIN_OBJECT_RADIUS_NORM;
    static constexpr float borderCompensation = 5;  // lock some pixels above field border for hypotheses
    size_t MAX_NUM_HYPOTHESES = 16;                    // max number of hypotheses generated
    static constexpr float ballSize = 0.1f;         // diameter of the ball.
    static constexpr float borderFactor =
            0.15f;  // how much border should be added around the ball when generating the hypotheses
    static constexpr int numAngles = 8;    // number of points for circle pattern
    static constexpr int BLOCK_SIZE = 32;  // divide image into blocks and calculate object size only block-wise
    static constexpr int blockSize = BLOCK_SIZE / IntegralImage::INTEGRAL_SCALE;
    static constexpr float OBJECT_RADIUS_SCALE = 1.5f;

    std::vector<ObjectHypothesis> hypoList;  // list with hypotheses results
    std::vector<point_2d> vec;               // vector with circle-pattern for position improvement

    IntegralImage* integralImg;

    const int rWidth;
    const int rHeight;
    const int numBlockX;
    const int numBlockY;
    std::vector<float> blockObjectRadius;
    int* adaptiveBlurImg;
    int* ratingImg;

    bool isDebugActive = false;
    uint8_t* debugImg = nullptr;

    void improvePositionAccuracy(std::vector<ObjectHypothesis>& selectedList);
    void improveHypothesesRatings(std::vector<ObjectHypothesis>& selectedList);
    std::vector<ObjectHypothesis> searchObjectHypotheses(const std::vector<int>& border);
    void selectBestHypotheses(std::vector<ObjectHypothesis>& maxList);
    void addSurroundingHypotheses(std::vector<ObjectHypothesis>& hypotheses);
    bool containsObject(const std::vector<ObjectHypothesis>& maxList, const ObjectHypothesis& hyp);
    void calculateBlockRating(const float objectRadius, const int px, const int py,
                              std::vector<ObjectHypothesis>& maxList, const std::vector<int>& border);
    void calculateBlockBlur(const float objectRadius, const int px, const int py);
    void calculateBlockRadii(CamPose& cam_pose);

    // integral image getArea function
    inline int getArea(const int px1, const int py1, const int px2, const int py2) {
        return (px2 - px1) * (py2 - py1);
    }

    void createIntegralImage(int* dataY, int* dataCr, int* iImage, int iWidth, int iHeight);
    point_2d getSubpixelPosition2(const int* data, int x, int y, const int width, const int height);

    void calculateBlockBlurBorder(const float objectRadius, const int px, const int py);
    void calculateBlockBlurSSERatingScale4(const float objectRadius, const int px, const int py);
    void calculateBlockBlurSSERatingScale2(const float objectRadius, const int px, const int py);
    void moveGridHypotheses(const std::vector<ObjectHypothesis>& hyp, std::vector<bool>& hypUsed, int gridSizeX,
                            int gridSizeY);

public:
    HypothesesGeneratorBlur(IntegralImage* integralImage, int8_t* lutCb, int8_t* lutCr, HtwkVisionConfig& config);
    ~HypothesesGeneratorBlur() override;

    void proceed(uint8_t* img, const std::vector<int>& fieldborder, CamPose& cam_pose,
                 IntegralImage* _integralImg) override;
    std::vector<ObjectHypothesis> getHypotheses() const override {
        return hypoList;
    }
    int* getRatingImg() override {
        return ratingImg;
    }

    uint8_t* getDebugImg() override {
        return debugImg;
    }

    void setDebugActive(bool active) override {
        isDebugActive = active;
    }
};

}  // namespace htwk

#endif  // HYPOTHESES_GENERATOR_BLUR_H
