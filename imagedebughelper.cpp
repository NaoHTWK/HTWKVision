#include "imagedebughelper.h"

#include <cstring>

void htwk::ImageDebugHelper::process(uint8_t *img) {
    if (isIntegralImageInputActive)
        drawIntegralImageInput(img);

    if (isDrawRatingImageActive)
        drawRatingImage(vision, img);

    if (isDrawObstacleInputParameterActive)
        vision.obstacleDetectionLowCam->drawInputParameter(img);

    if (isDrawObstacleResultActive)
        vision.obstacleDetectionLowCam->drawResult(img);

    if (isDrawBallDetectionLowerCamInputParameterActive)
        vision.lcImagePreprocessor->drawScaledImage(img);

    if (isDrawFieldBorderInputDataActive)
        vision.fieldBorderDetector->drawInputParameter(img);

    if (isDrawFieldBorderActive)
        vision.fieldBorderDetector->drawFieldBorder(img);

    if (isDrawUCBallHypGenScaledImage && vision.getHtwkVisionConfig().isUpperCam)
        vision.ucBallHypImagePreprocessor->drawScaledImage(img);

    if (isJerseyDetectionActive && vision.getHtwkVisionConfig().isUpperCam)
        vision.jerseyDetection->drawDebugOutput(img);
}

void htwk::ImageDebugHelper::drawIntegralImageInput(uint8_t *img) {
    uint8_t *destImg = new uint8_t[width * height * 2];
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int value = (getY(img, x, y) + 3 * getCr(img, x, y)) / 4 /* number of parts y+3*cr*/;
            setYCbCr(destImg, x, y, (uint8_t)std::min(value, 255), 128, 128);
        }
    }

    std::memcpy(img, destImg, width * height * 2);
    delete[] destImg;
}

void htwk::ImageDebugHelper::drawRatingImage(htwk::HTWKVision &vision, uint8_t *img) {
    std::memcpy(img, vision.hypothesesGenerator->getDebugImg(), width * height * 2);
}
