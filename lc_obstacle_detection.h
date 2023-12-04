#ifndef OBSTACLEDETECTIONLOWCAM_H
#define OBSTACLEDETECTIONLOWCAM_H

#include <map>
#include <vector>

#include "base_detector.h"
#include <integral_image.h>
#include <htwk_vision_config.h>
#include <tfliteexecuter.h>

namespace htwk {

class LowerCamObstacleDetection : protected BaseDetector
{
public:
    const int inputHeight;
    const int inputWidth;

    static constexpr int outputHeight = 8;
    static constexpr int outputWidth = 8;

    LowerCamObstacleDetection(int8_t* lutCb, int8_t* lutCr, HtwkVisionConfig& config);
    LowerCamObstacleDetection(const LowerCamObstacleDetection&) = delete;
    LowerCamObstacleDetection(const LowerCamObstacleDetection&&) = delete;
    LowerCamObstacleDetection& operator=(const LowerCamObstacleDetection&) = delete;
    LowerCamObstacleDetection& operator=(LowerCamObstacleDetection&&) = delete;
    ~LowerCamObstacleDetection();

    void proceed(float headYaw, uint8_t* img);
    std::vector<float> getInputParameter();
    const std::vector<float>& getDetectionResult(){return detectionResult;}

    bool isDetectionResultValid() const { return isOutputValid; }

    bool isShoulderInImage(float headYaw);

    void drawInputParameter(uint8_t* yuvImage);
    void drawResult(uint8_t* yuvImage);

private:
    std::map<int, int> histogram;
    size_t inputSize;

    float* input;
    TFLiteExecuter tflite;
    std::vector<float> detectionResult;

    bool shouldWeClassifyData;

    bool isOutputValid;

    void createInputData(uint8_t* img);
    void classifyData();
};

}//HTWK
#endif
