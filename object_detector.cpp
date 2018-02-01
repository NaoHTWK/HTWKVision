#include "object_detector.h"

namespace htwk {
ObjectDetector::ObjectDetector(int width, int height,
                               int8_t *lutCb, int8_t *lutCr,
                               BallFeatureExtractor* _featureExtractor,
                               const HtwkVisionConfig& config)
    : BallDetector(width, height, lutCb, lutCr)
    , MIN_BALL_PROB(config.ballProbabilityThreshold)
    , MIN_PENALTYSPOT_PROB(config.penaltyspotProbabilityThreshold)
    , MIN_FEET_PROB(config.feetProbabilityThreshold)
    , FEATURE_SIZE(config.objectDetectorPatchSize)
    , foundBall(false)
    , foundPenaltySpot(false)
    , foundFeet(0)
    , featureExtractor(_featureExtractor)
{
    caffe::Caffe::set_mode(caffe::Caffe::CPU);
    classifier.reset(new caffe::Net<float>(config.objectDetectorNetwork, caffe::TEST));
    classifier->CopyTrainedLayersFrom(config.objectDetectorModel);
    inputLayer = classifier->input_blobs()[0];
    inputLayer->Reshape(std::vector<int>{(int)config.hypothesisGeneratorMaxHypothesisCount, 1, inputLayer->height(), inputLayer->width()});
    classifier->Reshape();
    outputLayer = classifier->output_blobs()[0];
}

/*
 * detects the ball (if visible) and outputs its position
 */
void ObjectDetector::proceed(const uint8_t *img, std::vector<ObjectHypothesis> &hypoList)
{
    ratedBallHypotheses.clear();
    ratedPenaltySpotHypotheses.clear();
    ratedFeetHypotheses.clear();

    foundBall=false;
    foundPenaltySpot=false;
    foundFeet=0;

    float maxBallProb=MIN_BALL_PROB;
    float maxPenaltySpotProb=MIN_PENALTYSPOT_PROB;

    float* curInputPoint = inputLayer->mutable_cpu_data();

    for(const ObjectHypothesis& hyp : hypoList){
        featureExtractor->getFeature(hyp, img, FEATURE_SIZE, curInputPoint);
        curInputPoint += FEATURE_SIZE*FEATURE_SIZE;
    }

    classifier->Forward();

    const float* outputPos = outputLayer->cpu_data();
    const int outputOffset = outputLayer->shape(1);

    for(ObjectHypothesis& hyp : hypoList)
    {
        const float ballProb = outputPos[1];
        const float penaltySpotProb = outputPos[2];
        const float feetProb = outputPos[3];
        outputPos += outputOffset;

        if(ballProb > MIN_BALL_PROB){
            hyp.prob = ballProb;
            ratedBallHypotheses.push_back(hyp);
            if (ballProb > maxBallProb) {
                maxBallProb = ballProb;
                bestBallHypothesis=hyp;
            }
        }

        if(penaltySpotProb > MIN_PENALTYSPOT_PROB){
            hyp.prob = penaltySpotProb;
            ratedPenaltySpotHypotheses.push_back(hyp);
            if (penaltySpotProb > maxPenaltySpotProb) {
                maxPenaltySpotProb = penaltySpotProb;
                bestPenaltySpotHypothesis = hyp;
            }
        }

        if(feetProb > MIN_FEET_PROB){
            foundFeet++;
            hyp.prob = feetProb;
            ratedFeetHypotheses.push_back(hyp);
        }

    }

    foundBall = maxBallProb > MIN_BALL_PROB;
    foundPenaltySpot = maxPenaltySpotProb > MIN_PENALTYSPOT_PROB;
}

}  // namespace htwk
