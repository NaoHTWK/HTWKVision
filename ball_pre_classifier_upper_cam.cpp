#include "ball_pre_classifier_upper_cam.h"

#include <easy/profiler.h>
#include <stl_ext.h>

#include <cstring>
#include <thread>

namespace htwk {

BallPreClassifierUpperCam::BallPreClassifierUpperCam(int8_t* lutCb, int8_t* lutCr,
                                                     BallFeatureExtractor* featureExtractor,
                                                     HtwkVisionConfig &config)
    : BallDetector(lutCb, lutCr, config), featureExtractor(featureExtractor) {

    tflite.loadModelFromFile(
            config.tflitePath + "/uc-ball-small-classifier.tflite",
            {config.hypothesisGeneratorMaxHypothesisCount + config.ucBallHypGeneratorConfig.hypothesisCount,
             config.ballDetectorPatchSize, config.ballDetectorPatchSize, 1},
            config.ballPreClassifierUpperCamThreads);
}

/*
 * detects the ball (if visible) and outputs its position
 */
void BallPreClassifierUpperCam::proceed(const uint8_t* img, std::shared_ptr<FieldBorderDetector> fieldBorderDetector, std::vector<ObjectHypothesis>& hypoList) {
    Timer t("BallPreClassifierUpperCam", 50);
    EASY_FUNCTION(profiler::colors::Green);

    ratedBallHypotheses.clear();

    float maxBallProb = config.ballProbabilityThreshold;

    EASY_BLOCK("Get Feature");
    const auto& fieldBorder = fieldBorderDetector->getConvexFieldBorder();
    float* curInputPoint = tflite.getInputTensor();
    for (const ObjectHypothesis& hyp : hypoList) {        
        featureExtractor->getFeature(hyp, img, config.ballDetectorPatchSize, curInputPoint);
        curInputPoint += config.ballDetectorPatchSize * config.ballDetectorPatchSize;
    }
    EASY_END_BLOCK;

    EASY_BLOCK("TFlite Small BallDetector");
    tflite.execute();
    EASY_END_BLOCK;

    const float* outputPosBall = tflite.getOutputTensor();
    const int outputOffset = 2;

    allHypothesesWithProb = hypoList;

    bestBallHypothesis = std::nullopt;
    for (size_t i = 0; i < hypoList.size(); i++) {
        ObjectHypothesis& hypProb = allHypothesesWithProb[i];

        const float ballProb = outputPosBall[1];
        outputPosBall += outputOffset;

        hypProb.prob = ballProb;

        // Remove every hypotheses which is 10% above the field border
        int x = clamp(hypProb.x, 0, width-1);
        if (hypProb.y < fieldBorder[x] - (height * 0.1f)) {
            hypProb.prob = 0;
        }

        if (ballProb > config.ballProbabilityThreshold) {
            ratedBallHypotheses.push_back(hypProb);

            if (ballProb > maxBallProb) {
                maxBallProb = ballProb;
                bestBallHypothesis = hypProb;
            }
        }
    }
}

}  // namespace htwk
