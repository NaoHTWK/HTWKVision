#include "robot_detector.h"

namespace htwk {

RobotDetector::RobotDetector(int width, int height, int8_t *lutCb, int8_t *lutCr,
                             BallFeatureExtractor* _featureExtractor,
                             const HtwkVisionConfig& config)
   : BaseDetector(width, height, lutCb, lutCr)
   , MIN_FEET_PROB(config.feetProbabilityThreshold)
   , FEATURE_SIZE(config.objectDetectorPatchSize)
   , foundFeet(0)
   , featureExtractor(_featureExtractor)
{
    caffe::Caffe::set_mode(caffe::Caffe::CPU);
//    classifier.reset(new caffe::Net<float>("./data/feet_classifier.prototxt", caffe::TEST));
//    classifier->CopyTrainedLayersFrom("./data/feet_classifier.caffemodel");
    classifier.reset(new caffe::Net<float>(config.objectDetectorNetwork, caffe::TEST));
    classifier->CopyTrainedLayersFrom(config.objectDetectorModel);

    inputLayer = classifier->input_blobs()[0];
    inputLayer->Reshape(std::vector<int>{(int)config.hypothesisGeneratorMaxHypothesisCount, 1, inputLayer->height(), inputLayer->width()});
    classifier->Reshape();
    outputLayer = classifier->output_blobs()[0];
}

/*
 * detects robots (if visible) and outputs its position
 */
void RobotDetector::proceed(uint8_t *img, std::vector<ObjectHypothesis> &hypoList){
    ratedHypothesis.clear();
    foundFeet=0;

    float* curInputPoint = inputLayer->mutable_cpu_data();
    for(const ObjectHypothesis& hyp : hypoList){
        featureExtractor->getFeature(hyp, img, FEATURE_SIZE, curInputPoint);
        curInputPoint += FEATURE_SIZE*FEATURE_SIZE;
    }
    classifier->Forward();

    const float* outputPos = outputLayer->cpu_data();
    const int outputOffset = outputLayer->shape(1);

    for(ObjectHypothesis &hyp:hypoList){
        const float prob = outputPos[1];
        outputPos += outputOffset;

        if(prob > MIN_FEET_PROB){
            foundFeet++;
            hyp.prob = prob;
            ratedHypothesis.push_back(hyp);
        }
    }
}
}  // namespace htwk

