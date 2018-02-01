#ifndef HTWK_VISION_H
#define HTWK_VISION_H

#include <ctime>
#include <vector>
#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <fstream>

#include "ball_detector.h"
#include "ball_detector_legacy.h"
#include "ball_feature_extractor.h"
#include "field_color_detector.h"
#include "feet_detector.h"
#include "field_detector.h"
#include "goal_detector.h"
#include "htwk_vision_config.h"
#include "integral_image.h"
#include "jersey_color_detector.h"
#include "line_detector.h"
#include "near_obstacle_detector.h"
#include "object_detector.h"
#include "ransac_ellipse_fitter.h"
#include "region_classifier.h"
#include "robot_area_detector.h"
#include "robot_classifier.h"
#include "robot_detector.h"
#include "hypotheses_generator_blocks.h"

namespace htwk {

/**
 * This module recognizes robocup specific objects in an image with 'width' * 'height' pixels in yuv422 format.
 * Recognition Overview:
 * FieldColorDetector     : detects the yCbCr color of the playing field in the image
 * FieldDetector          : decides which pixels belonging to the playing-field by modelling the fieldborder with up to two lines
 * RegionClassifier       : scans the image with vertical and horizontal scanlines and divides it into homogeneous scanline-segments
 *                          and classifies every region into 'carpet-green', 'line-white' or 'other'
 * GoalDetector           : detects goal posts in the image
 * LineDetector           : scans image for lines (straight groups of line segments from the RegionClassifier)
 * BallDetector           : detects the ball (if visible) and outputs its position
 * FeetDetector           : detects the own feets (if visible) and returns the position between the toes
 * JersyColorDetector     : detects the color of the given robot
 * RansacEllipseFitter    : trys to find an ellipse in the image
 * VisionDebugger         : used for visualising the recognition results (writes directly into the camera-image)
 * RobotAreaDetector      : detects possible rects in the image, where a other robot could be
 * RobotClassifier        : classifies every detected possible robot-rect to decide, if it is really a robot (to prevent false positives)
 * NearObstableDetector   : find obstacles
 */
class HTWKVision {
private:
    int8_t *lutCb = nullptr;
    int8_t *lutCr = nullptr;
    const int width;
//    int height;
    void createAdressLookups();
    std::vector<RobotClassifierResult> resultRobotClassifier;

    static constexpr bool enableProfiling = false;
    void createProfilingStats();

    inline void getTime(timespec& t) { clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t); }
    inline void writeProfilHeader(const std::string& fileName);
    inline void writeData(std::ofstream &file, const std::string& sep, float min, float avg, float max){
        file << min << sep << avg << sep << max << sep;
    }
    inline void writeHead(std::ofstream &file, const std::string& sep, const std::string& name){
        file << sep << name << sep << sep;
    }

    timespec tFieldColorDetector;
    timespec tRegionClassifier;
    timespec tFieldDetector;
    timespec tLineDetector;
    timespec tGoalDetector;
    timespec tHypoGenerator;
    timespec tBallDetector;
    timespec tFeetDetector;
    timespec tNearObstacleDetector;
    timespec tEllipseFitter;
    timespec tRobotHypotheses;
    timespec tRobotClassifier;
    timespec tJersey;
    timespec tIntegralImage;
    timespec tEnd;

    float cntFieldColorDetector = 0;
    float cntRegionClassifier = 0;
    float cntFieldDetector = 0;
    float cntLineDetector = 0;
    float cntGoalDetector = 0;
    float cntHypoGenerator = 0;
    float cntBallDetector = 0;
    float cntFeetDetector = 0;
    float cntNearObstacleDetector = 0;
    float cntEllipseFitter = 0;
    float cntRobotHypotheses = 0;
    float cntRobotClassifier = 0;
    float cntJersey = 0;
    float cntCreateIntegralImage = 0;
    float cntTotal = 0;

    float maxFieldColorDetector = 0;
    float maxRegionClassifier = 0;
    float maxFieldDetector = 0;
    float maxLineDetector = 0;
    float maxGoalDetector = 0;
    float maxHypoGenerator = 0;
    float maxBallDetector = 0;
    float maxFeetDetector = 0;
    float maxNearObstacleDetector = 0;
    float maxEllipseFitter = 0;
    float maxRobotHypotheses = 0;
    float maxRobotClassifier = 0;
    float maxJersey = 0;
    float maxCreateIntegralImage = 0;
    float maxTotal = 0;

    float minFieldColorDetector = 0;
    float minRegionClassifier = 0;
    float minFieldDetector = 0;
    float minLineDetector = 0;
    float minGoalDetector = 0;
    float minHypoGenerator = 0;
    float minBallDetector = 0;
    float minFeetDetector = 0;
    float minNearObstacleDetector = 0;
    float minEllipseFitter = 0;
    float minRobotHypotheses = 0;
    float minRobotClassifier = 0;
    float minJersey = 0;
    float minCreateIntegralImage = 0;
    float minTotal = 0;

    int cntImage = 0;

    HtwkVisionConfig config;

public:
    FieldColorDetector *    fieldColorDetector = nullptr;
    FieldDetector *         fieldDetector = nullptr;
    RegionClassifier*       regionClassifier = nullptr;
    LineDetector*           lineDetector = nullptr;
    GoalDetector*           goalDetector = nullptr;
    BallFeatureExtractor*   ballFeatureExtractor = nullptr;
    BallDetector*           ballDetector = nullptr;
    RansacEllipseFitter*    ellipseFitter = nullptr;
    FeetDetector*           feetDetector = nullptr;
    RobotAreaDetector*      robotAreaDetector = nullptr;
    RobotClassifier*        robotClassifier = nullptr;
    NearObstacleDetector*   nearObstacleDetector = nullptr;
    JerseyColorDetector*    jerseyColorDetector = nullptr;
    IntegralImage*          integralImage = nullptr;
    HypothesesGenerator*    hypothesesGenerator = nullptr;
    ObjectDetector*         objectDetector = nullptr;
    RobotDetector*          robotDetector  = nullptr;

    HTWKVision(int width, int height, HtwkVisionConfig  _config);
    HTWKVision(HTWKVision& h) = delete;
    HTWKVision(HTWKVision&& h) = delete;
    HTWKVision& operator=(const HTWKVision&) = delete;
    ~HTWKVision();

    void proceed(uint8_t *img, bool use_feet_detection, float pitch, float roll);

    /**elements are in pixelcoordinates**/
    std::vector<RobotClassifierResult> getRobotClassifierResult() const { return resultRobotClassifier; }
    void printProfilingResults(bool isUpperCam);
    void writeProfilingFile(const std::string& fileName, const std::string& imageName);
    void resetProfilingStats(bool isUpperCam);

    HtwkVisionConfig& getHtwkVisionConfig() { return config; }
};

}  // namespace htwk

#endif  // HTWK_VISION_H

