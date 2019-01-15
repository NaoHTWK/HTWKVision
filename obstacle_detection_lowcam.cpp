#include "obstacle_detection_lowcam.h"

#include <fast_math.h>

namespace htwk {

ObstacleDetectionLowCam::ObstacleDetectionLowCam(int width, int height, int8_t *lutCb, int8_t *lutCr, const HtwkVisionConfig& config)
    : BaseDetector(width, height, lutCb, lutCr),
      inputHeight(config.obstacleDetectionInputHeight),
      inputWidth(config.obstacleDetectionInputWidth),
      inputParameter(inputHeight*inputWidth),
      detectionResult(outputWidth*outputHeight),
      shouldWeClassifyData(config.obstacleClassifyData)
{
    if(shouldWeClassifyData) {
        caffe::Caffe::set_mode(caffe::Caffe::CPU);
        classifier.reset(new caffe::Net<float>(config.obstacleDetectorNetwork, caffe::TEST));
        classifier->CopyTrainedLayersFrom(config.obstacleDetectorModel);
        inputLayer = classifier->input_blobs()[0];
        inputLayer->Reshape(std::vector<int>{(int)config.obstacleBatchSize, 1, inputLayer->height(), inputLayer->width()});
        classifier->Reshape();
        outputLayer = classifier->output_blobs()[0];
    }
}

ObstacleDetectionLowCam::~ObstacleDetectionLowCam() {
}

bool ObstacleDetectionLowCam::isShoulderInImage(float headYaw) {
    const float weight = 5.38412472f;
    const float bias = -4.4397172f;

    return 1.f/(1.f + std::exp(-1.f*(std::fabs(headYaw)*weight)-bias)) > 0.25f;
}

void ObstacleDetectionLowCam::proceed(float headYaw, IntegralImage* integralImg){
    isOutputValid = false;
    histogramValid = false;

    // Don't process image if it's a lost cause.
    if(isShoulderInImage(headYaw)) {
        for(int i = 0; i < detectionResult.size(); i++)
            detectionResult[i] = 0.f;
        return;
    }

    histogram.clear();

    createInputData(integralImg, IntegralImage::iWidth, IntegralImage::iHeight);
    normalizeInputData(IntegralImage::iWidth, IntegralImage::iHeight);

    histogramValid = true;

    if(shouldWeClassifyData) {
        isOutputValid = true;
        classifyData();
//        for(int x=0;x<2*outputHeight*outputWidth;x++){
//            detectionResult[x]=clamp(0.f,detectionResult[x],1.f);
//        }
    }
}


void ObstacleDetectionLowCam::classifyData(){
    float* curInputPoint = inputLayer->mutable_cpu_data();
    memcpy(curInputPoint, inputParameter.data(), sizeof(float)*inputParameter.size());

    classifier->Forward();

    const float* outputPos = outputLayer->cpu_data();

    int offset = outputHeight*outputWidth;
    memcpy(detectionResult.data(), outputPos+offset, sizeof(float)*detectionResult.size());
}

/* 1. dynamische blockgröße erstellen
 *    a. unterschiedliche anzahl an zeilen und spalten
 *    b. änderung des eingangsbilds (größe)
 * 2. blöcke im integralbild analysieren
 *    a. dabei die blockwerte festhalten => input parameter
 */
void ObstacleDetectionLowCam::createInputData(IntegralImage* integralImg, int width, int height){
    const int blockWidth=width/inputWidth;
    const int blockHeight=height/inputHeight;
    const int blockInnerArea=blockWidth*blockHeight;
    for (int y=0;y<inputHeight;y++){
        int yTop    = y*blockHeight;
        int yBottom = yTop+blockHeight-1;
        for (int x=0;x<inputWidth;x++){
            int xLeft  = x*blockWidth;
            int xRight = xLeft+blockWidth-1;
            const int param = integralImg->getIntegralValue(xLeft,yTop,xRight,yBottom)/blockInnerArea;
            histogram[param]++;            
            inputParameter[x+y*inputWidth] = param;
        }
    }
}

void ObstacleDetectionLowCam::normalizeInputData(int width, int height){
    constexpr int divisionAddOn = 100;
    const int histogramSum=inputWidth*inputHeight;
    const int min = calculatePercental((int)(histogramSum*0.05f));
    const int max = calculatePercental((int)(histogramSum*0.95f));
//    printf("min_: %d | max_: %d\n", min, max);

//    const int minOld = histogram.begin()->first+(int)(histogram.begin()->first*0.05f);
//    const int maxOld = histogram.rbegin()->first-(int)(histogram.rbegin()->first*0.05f);
//    printf("minO: %d | maxO: %d\n", minOld, maxOld);

    for (int y=0;y<inputHeight;y++){
        for (int x=0;x<inputWidth;x++){
            const int division = std::max(max - min , divisionAddOn);
            const float param = (float)(inputParameter[x+y*inputWidth] - min) / division;
            inputParameter[x+y*inputWidth]=clamp(0.f,param,1.f);
        }
    }
}

int ObstacleDetectionLowCam::calculatePercental(int threshold){
    int sum=0;
    int percental=histogram.rbegin()->first;
    for (auto const& entry : histogram){
        sum+=entry.second;
        if (sum>threshold){
            percental=entry.first;
            break;
        }
    }
    return percental;
}

}//HTWK
