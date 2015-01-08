#ifndef __HTWK_VISION_H__
#define __HTWK_VISION_H__

#include <vector>
#include <stdint.h>

#include <ball_detector.h>
#include <field_color_detector.h>
#include <feet_detector.h>
#include <field_detector.h>
#include <goal_detector.h>
#include <line_detector.h>
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
 * RansacEllipseFitter    : trys to find an ellipse in the image
 * VisionDebugger         : used for visualising the recognition results (writes directly into the camera-image)
 * RobotAreaDetector      : detects possible rects in the image, where a other robot could be
 * RobotClassifier        : classifies every detected possible robot-rect to decide, if it is really a robot (to prevent false positives)
 */
class HTWKVision {
private:
    int *lutCb;
    int *lutCr;
  int width;
  int height;
    void createAdressLookups();
    std::vector<classifierResult> resultRobotClassifier;

    HTWKVision(HTWKVision& h);
    void operator=(HTWKVision const&);

public:
    FieldColorDetector *fieldColorDetector;
    FieldDetector *fieldDetector;
    RegionClassifier *regionClassifier;
    LineDetector *lineDetector;
    GoalDetector *goalDetector;
    BallDetector *ballDetector;
    RansacEllipseFitter *ellipseFitter;
    FeetDetector *feetDetector;
    RobotAreaDetector *robotAreaDetector;
    RobotClassifier   *robotClassifier;
    HTWKVision(int width, int height);
    ~HTWKVision();
    void proceed(uint8_t *img, bool is_upper, bool useFeetDetection);
    /**elements are in pixelcoordinates**/
    std::vector<classifierResult> getRobotClassifierResult() const { return resultRobotClassifier; };
};

}  // namespace htwk

#endif  // __HTWK_VISION_H__

