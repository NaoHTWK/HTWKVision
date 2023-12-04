#ifndef HTWK_VISION_CONFIG_H
#define HTWK_VISION_CONFIG_H

#include <string>

namespace htwk {

struct UCBallHypothesesGeneratorConfig {
    int scaledImageWidth = 160;
    int scaledImageHeight = 120;
    int patchWidth = 40;
    int patchHeight = 30;
    int hypothesisCount = 16;
    bool classifyHypData = true;
    std::string model = "uc-ball-hyp-generator.tflite";
};

struct UCBallLargeClassifierConfig {
    int patchSize = 24;
    int camHypothesesCount = 2;
    bool classifyData = true;
    float smallBallProbabilityThreshold = 0.94;
    float probabilityThreshold = 0.984694f;  // ;hitrate:0.745607;prec.:0.999854
};

struct UCGoalPostDetectorConfig {
    int scaledImageWidth = 80;
    int scaledImageHeight = 60;
    int patchesX = 4;
    int patchesY = 1;
    int threads = 1;
    bool runDetector = true;
    float probabilityThreshold = 0.99;  // hitrate:0.731559;prec.:0.980204
    std::string model = "uc-goalpost-detector.tflite";
};

struct UCCenterCirclePointDetectorConfig {
    int scaledImageWidth = 80;
    int scaledImageHeight = 60;
    int patchesX = 4;
    int patchesY = 1;
    int threads = 1;
    bool runDetector = true;
    float probabilityThreshold = 0.9859;  // hitrate:0.513299;prec.:0.991166
    std::string model = "uc-centercirclepoint-detector.tflite";
};

struct LCCenterCirclePointDetectorConfig {
    int scaledImageWidth = 40;
    int scaledImageHeight = 30;
    bool runDetector = true;
    float probabilityThresholdCenter = 0.998051;  // hitrate:0.294246;prec.:0.998521
    float probabilityThresholdSide = 0.99611;   // hitrate:0.541792;prec.:0.997548
    std::string model_center = "lc-centercirclepoint-center-detector.tflite";
    std::string model_side = "lc-centercirclepoint-side-detector.tflite";
};

struct LCObjectDetectorConfig {
    int scaledImageWidth = 40;
    int scaledImageHeight = 30;
    int patchSize = 24;
    bool classifyHypData = true;
    bool classifyObjData = true;

    float ballProbabilityThreshold = 0.95f;
    float penaltySpotProbabilityThreshold = 0.9817f;

    std::string hypGenModelBall = "lc-object-hyp-gen-ball.tflite";
    std::string hypGenModelPenatlySpot = "lc-object-hyp-gen-penaltyspot.tflite";
};

struct UCPenaltySpotClassifierConfig {
    // From which probability on a hypothesis is for sure a penaltyspot
    float probThreshold = 0.9889;  // ;hitrate:0.458629;prec.:0.96101

    // Which size are patches that the object detector uses
    int patchSize = 16;
};

struct UCDirtyCameraDetector {
    bool run = false;
    int scaledImageWidth = 160;
    int scaledImageHeight = 120;
    std::string model = "uc-dirty-cam-detector.tflite";
    int threads = 2;
};

struct LCScrambledCameraDetector {
    bool run = false;
    int scaledImageWidth = 40;
    int scaledImageHeight = 30;
    std::string model = "lc-scrambled-cam-detector.tflite";
    int threads = 1;
};

/**
 * @brief The HtwkVisionConfig struct contains config variables for the vision.
 *
 * The default values should be working in a normal game. If you need other values
 * please create an instance and construct an HTWKVision object with it.
 */
struct HtwkVisionConfig {
    int width = 640;
    int height = 480;

    // From which probability on a hypothesis is for sure a ball
    float ballProbabilityThreshold = 0.99f;

    // Which size are patches that the ball detector uses
    int ballDetectorPatchSize = 12;

    int hypothesisGeneratorMaxHypothesisCount = 40;

    int ballPreClassifierUpperCamThreads = 2;

    bool isUpperCam = true;

    // Path where all tflite models are stored.
    std::string tflitePath;

    // How many rows and cols are used by the obstacle detection
    int obstacleDetectionInputHeight = 32;
    int obstacleDetectionInputWidth = 32;
    int obstacleBatchSize = 1;
    bool obstacleClassifyData = true;

    int fieldBorderHeight = 30;
    int fieldBorderWidth = 40;
    int fieldBorderClassifyData = true;

    LCObjectDetectorConfig lcObjectDetectorConfig;
    UCBallHypothesesGeneratorConfig ucBallHypGeneratorConfig;
    UCBallLargeClassifierConfig ucBallLargeClassifierConfig;
    UCGoalPostDetectorConfig ucGoalPostDetectorConfig;
    UCCenterCirclePointDetectorConfig ucCenterCirclePointDetectorConfig;
    LCCenterCirclePointDetectorConfig lcCenterCirclePointDetectorConfig;
    UCPenaltySpotClassifierConfig ucPenaltySpotClassifierConfig;
    UCDirtyCameraDetector ucDirtyCameraDetector;
    LCScrambledCameraDetector lcScrambledCameraDetector;

    bool onlyLocalization = false;

    size_t imageMemorySize() const {
        return 2 * width * height + 16 /* SSE alignment */;
    }

#ifdef VISION_ACTIVATE_VISUALIZATION
    static constexpr bool activate_visualization = true;
#else
    static constexpr bool activate_visualization = false;
#endif

    HtwkVisionConfig();
};

}  // namespace htwk

#endif  // HTWK_VISION_CONFIG_H
