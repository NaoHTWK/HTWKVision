#ifndef IMAGEDEBUGHELPER_H
#define IMAGEDEBUGHELPER_H

#include <base_detector.h>
#include <htwk_vision.h>

namespace htwk {

class ImageDebugHelper : public BaseDetector {
public:
    ImageDebugHelper(HTWKVision& vision, HtwkVisionConfig& config)
        : BaseDetector(vision.getLutCb(), vision.getLutCr(), config), vision(vision) {}

    void process(uint8_t* img);

    void setIntegralImageInputActive(bool active) {
        isIntegralImageInputActive = active;
    }

    void setDrawRatingImageActive(bool active) {
        isDrawRatingImageActive = active;
        vision.hypothesesGenerator->setDebugActive(active);
    }

    void setDrawObstacleInputParameterActive(bool active) {
        isDrawObstacleInputParameterActive = active;
    }

    void setBallDetectionLowerCamInputParameterActive(bool active) {
        isDrawBallDetectionLowerCamInputParameterActive = active;
    }

    void setDrawObstacleResultActive(bool active) {
        isDrawObstacleResultActive = active;
    }

    void setDrawFieldBorderInputData(bool active) {
        isDrawFieldBorderInputDataActive = active;
    }

    void setFieldBorderData(bool active) {
        isDrawFieldBorderActive = active;
    }

    void setDrawUCBallHypGenScaledImage(bool active) {
        isDrawUCBallHypGenScaledImage = active;
    }

    void setJerseyDetection(bool active) {
        isJerseyDetectionActive = active;
    }

private:
    HTWKVision& vision;

    bool isIntegralImageInputActive = false;
    bool isDrawRatingImageActive = false;
    bool isDrawObstacleInputParameterActive = false;
    bool isDrawObstacleResultActive = false;
    bool isDrawBallDetectionLowerCamInputParameterActive = false;

    bool isDrawFieldBorderInputDataActive = false;
    bool isDrawFieldBorderActive = false;

    bool isDrawUCBallHypGenScaledImage = false;
    bool isJerseyDetectionActive = false;

    void drawIntegralImageInput(uint8_t* img);
    void drawRatingImage(HTWKVision& vision, uint8_t* img);
};

}  // namespace htwk
#endif  // IMAGEDEBUGHELPER_H
