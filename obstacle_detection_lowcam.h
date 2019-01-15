#ifndef OBSTACLEDETECTIONLOWCAM_H
#define OBSTACLEDETECTIONLOWCAM_H

#include "base_detector.h"
#include <integral_image.h>
#include <htwk_vision_config.h>

#include <vector>
#include <map>

#include <caffe/caffe.hpp>

namespace htwk {

class ObstacleDetectionLowCam : protected BaseDetector
{
public:
    const int inputHeight;
    const int inputWidth;

    static constexpr int outputHeight = 3;
    static constexpr int outputWidth = 3;

    ObstacleDetectionLowCam(int width, int height, int8_t* lutCb, int8_t* lutCr, const HtwkVisionConfig& config);
    ~ObstacleDetectionLowCam();

    void proceed(float headYaw, IntegralImage* integralImg);
    const std::vector<float>& getInputParameter(){return inputParameter;}
    const std::map<int,int>& getHistogram(){return histogram;}
    const std::vector<float>& getDetectionResult(){return detectionResult;}

    bool isDetectionResultValid() const { return isOutputValid; }

    // remove this later?
    bool isHistogramValid() const { return histogramValid; }

    bool isShoulderInImage(float headYaw);

private:
    std::vector<float> inputParameter;
    std::map<int,int> histogram;

    std::shared_ptr<caffe::Net<float>> classifier;
    caffe::Blob<float>* inputLayer;
    caffe::Blob<float>* outputLayer;

    std::vector<float> detectionResult;

    bool shouldWeClassifyData;

    bool isOutputValid;
    bool histogramValid;

    void createInputData(IntegralImage* integralImg, int width, int height);
    void classifyData();
    void normalizeInputData(int width, int height);
    int calculatePercental(int threshold);
};

}//HTWK
#endif
