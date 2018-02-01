#ifndef HTWK_VISION_CONFIG_H
#define HTWK_VISION_CONFIG_H

#include <string>

namespace htwk {

/**
 * @brief The HtwkVisionConfig struct contains config variables for the vision.
 *
 * The default values should be working in a normal game. If you need other values
 * please create an instance and construct an HTWKVision object with it.
 */
struct HtwkVisionConfig {
    // From which probability on a hypothesis is for sure a ball
    float ballProbabilityThreshold = 0.967f;

    // From which probability on a hypothesis is for sure a penaltyspot
    float penaltyspotProbabilityThreshold = 0.967f;

    // From which probability on a hypothesis is for sure a feet
    float feetProbabilityThreshold = 0.964f;

    // Which size are patches that the object detector uses
    int objectDetectorPatchSize = 12;
    std::string objectDetectorNetwork = "./data/object_classifier_runtime.prototxt";
    std::string objectDetectorModel   = "./data/object_classifier_runtime.caffemodel";
    bool activateObjectDetector = true;

    size_t hypothesisGeneratorMaxHypothesisCount = 16;

    bool isUpperCam = true;
};

} // htwk

#endif // HTWK_VISION_CONFIG_H
