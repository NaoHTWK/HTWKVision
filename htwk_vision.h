#ifndef __HTWK_VISION_H__
#define __HTWK_VISION_H__

#include <chrono>
#include <vector>
#include <stdint.h>

#include <ball_detector.h>
#include <field_color_detector.h>
#include <feet_detector.h>
#include <field_detector.h>
#include <goal_detector.h>
#include <jersey_color_detector.h>
#include <line_detector.h>
#include <near_obstacle_detector.h>
#include <ransac_ellipse_fitter.h>
#include <region_classifier.h>
#include <robot_area_detector.h>
#include <robot_classifier.h>

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
    int8_t *lutCb;
    int8_t *lutCr;
    int width;
    int height;
    void createAdressLookups();
    std::vector<RobotClassifierResult> resultRobotClassifier;

    HTWKVision(HTWKVision& h);
    void operator=(HTWKVision const&);

    static constexpr bool enableProfiling = false;

    std::chrono::high_resolution_clock::time_point tFieldColorDetector;
    std::chrono::high_resolution_clock::time_point tRegionClassifier;
    std::chrono::high_resolution_clock::time_point tFieldDetector;
    std::chrono::high_resolution_clock::time_point tLineDetector;
    std::chrono::high_resolution_clock::time_point tGoalDetector;
    std::chrono::high_resolution_clock::time_point tBallDetector;
    std::chrono::high_resolution_clock::time_point tFeetDetector;
    std::chrono::high_resolution_clock::time_point tNearObstacleDetector;
    std::chrono::high_resolution_clock::time_point tEllipseFitter;
    std::chrono::high_resolution_clock::time_point tRobotArea;
    std::chrono::high_resolution_clock::time_point tJersey;


    double cntFieldColorDetector = 0;
    double cntRegionClassifier = 0;
    double cntFieldDetector = 0;
    double cntLineDetector = 0;
    double cntGoalDetector = 0;
    double cntBallDetector = 0;
    double cntFeetDetector = 0;
    double cntNearObstacleDetector = 0;
    double cntEllipseFitter = 0;
    double cntRobotArea = 0;
    double cntJersey = 0;
    double cntCreateIntegralImage = 0;
    double cntTotal = 0;

    int cntImages = 0;

    void printProfilingResults();

public:
    FieldColorDetector *    fieldColorDetector;
    FieldDetector *         fieldDetector;
    RegionClassifier*       regionClassifier;
    LineDetector*           lineDetector;
    GoalDetector*           goalDetector;
    BallDetector*           ballDetector;
    RansacEllipseFitter*    ellipseFitter;
    FeetDetector*           feetDetector;
    RobotAreaDetector*      robotAreaDetector;
    RobotClassifier*        robotClassifier;
    NearObstacleDetector*   nearObstacleDetector;
    JerseyColorDetector*    jerseyColorDetector;

    HTWKVision(int width, int height);
    ~HTWKVision();

    void proceed(uint8_t *img, bool is_upper, bool useFeetDetection);

    /**elements are in pixelcoordinates**/
    std::vector<RobotClassifierResult> getRobotClassifierResult() const { return resultRobotClassifier; }
};

}  // namespace htwk

#endif  // __HTWK_VISION_H__

