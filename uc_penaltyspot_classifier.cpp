#include "uc_penaltyspot_classifier.h"

#include <easy/profiler.h>

#include <cstring>

namespace htwk {
UpperCamPenaltySpotClassifier::UpperCamPenaltySpotClassifier(int8_t* lutCb, int8_t* lutCr, BallFeatureExtractor* _featureExtractor,
                               HtwkVisionConfig &config)
    : BaseDetector(lutCb, lutCr, config), featureExtractor(_featureExtractor) {

    patchSize = config.ucPenaltySpotClassifierConfig.patchSize;
    tflitePenaltyspot.loadModelFromFile(config.tflitePath + "/uc-penaltyspot-classifier.tflite",
                                        {config.hypothesisGeneratorMaxHypothesisCount, patchSize,
                                         patchSize, 1});
}

/*
 * detects the ball (if visible) and outputs its position
 */
void UpperCamPenaltySpotClassifier::proceed(const uint8_t* img, const std::vector<ObjectHypothesis>& hypoList) {
    Timer t("ObjectDetector", 50);
    EASY_FUNCTION(profiler::colors::Green);

    ratedPenaltySpotHypotheses.clear();
    penaltySpotHypotheses = std::nullopt;

    float maxPenaltySpotProb = config.ucPenaltySpotClassifierConfig.probThreshold;
    float* curInputPointPenaltySpot = tflitePenaltyspot.getInputTensor();

    EASY_BLOCK("Get Feature");
    for (const ObjectHypothesis& hyp : hypoList) {
        featureExtractor->getFeature(hyp, img, patchSize, curInputPointPenaltySpot);
        curInputPointPenaltySpot += patchSize * patchSize;
    }
    EASY_END_BLOCK;

    EASY_BLOCK("TFLite PenaltySpot Execute");
    tflitePenaltyspot.execute();
    EASY_END_BLOCK;

    const float* outputPosPenaltyspot = tflitePenaltyspot.getOutputTensor();
    const int outputOffsetPenaltyspot = 1;

    for (ObjectHypothesis hyp : hypoList) {
        const float penaltySpotProb = outputPosPenaltyspot[0];
        outputPosPenaltyspot += outputOffsetPenaltyspot;

        hyp.prob = penaltySpotProb;
        ratedPenaltySpotHypotheses.push_back(hyp);
        if (penaltySpotProb > config.ucPenaltySpotClassifierConfig.probThreshold) {
            if (penaltySpotProb > maxPenaltySpotProb) {
                maxPenaltySpotProb = penaltySpotProb;
                penaltySpotHypotheses = hyp;
            }
        }
    }
}

}  // namespace htwk
