#ifndef HTWK_VISION_H
#define HTWK_VISION_H

#include <async.h>
#include <ball_classifier_upper_cam.h>
#include <ball_detector.h>
#include <ball_feature_extractor.h>
#include <ball_pre_classifier_upper_cam.h>
#include <field_border_detector.h>
#include <field_color_detector.h>
#include <htwk_vision_config.h>
#include <hypotheses_generator.h>
#include <image_preprocessor.h>
#include <integral_image.h>
#include <jersey_detection.h>
#include <lc_centercirclepoints_detector.h>
#include <lc_obstacle_detection.h>
#include <lc_scrambled_camera_detector.h>
#include <line_detector.h>
#include <localization_utils.h>
#include <object_detector_lowercam.h>
#include <object_detector_lowercam_hyp_gen.h>
#include <penaltyspot_detector.h>
#include <ransac_ellipse_fitter.h>
#include <region_classifier.h>
#include <uc_ball_hyp_gen.h>
#include <uc_centercirclepoints_detector.h>
#include <uc_goalpost_detector.h>
#include <uc_penaltyspot_classifier.h>
#include <uc_robot_detector.h>
#include <uc_dirty_camera_detector.h>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>

namespace htwk {

class ImageDebugHelper;

/**
 * This module recognizes robocup specific objects in an image with 'width' * 'height' pixels in yuv422 format.
 * Recognition Overview:
 * FieldColorDetector     : detects the yCbCr color of the playing field in the image
 * FieldBorderDetector    : decides which pixels belonging to the playing-field by modelling the fieldborder with up to
 *                          two lines
 * RegionClassifier       : scans the image with vertical and horizontal scanlines and divides it into
 *                          homogeneous scanline-segments and classifies every region into 'carpet-green', 'line-white'
 *                          or 'other'
 * LineDetector           : scans image for lines (straight groups of line segments from the RegionClassifier)
 * BallDetector           : detects the ball (if visible) and outputs its position
 * RansacEllipseFitter    : trys to find an ellipse in the image
 * VisionDebugger         : used for visualising the recognition results (writes directly into the camera-image)
 */
class HTWKVision {
private:
    int8_t* lutCb = nullptr;
    int8_t* lutCr = nullptr;
    void createAdressLookups();

    HtwkVisionConfig config;
    ThreadPool* thread_pool;

    std::vector<float> stuckCameraReferenceImage;

public:
    FieldColorDetector* fieldColorDetector = nullptr;
    std::shared_ptr<FieldBorderDetector> fieldBorderDetector = nullptr;
    RegionClassifier* regionClassifier = nullptr;
    LineDetector* lineDetector = nullptr;
    RansacEllipseFitter* ellipseFitter = nullptr;
    IntegralImage* integralImage = nullptr;
    HypothesesGenerator* hypothesesGenerator = nullptr;
    LowerCamObstacleDetection* obstacleDetectionLowCam = nullptr;

    BallFeatureExtractor* ballFeatureExtractor = nullptr;

    std::shared_ptr<ImagePreprocessor> ucBallHypImagePreprocessor;
    std::shared_ptr<UpperCamBallHypothesesGenerator> ucBallHypGenerator;

    std::shared_ptr<ImagePreprocessor> ucImagePreprocessor;
    std::shared_ptr<UpperCamGoalPostDetector> ucGoalPostDetector;
    std::shared_ptr<UpperCamCenterCirclePointDetector> ucCenterCirclePointDetector;
    std::shared_ptr<LowerCamCenterCirclePointDetector> lcCenterCirclePointDetectorCenter;
    std::shared_ptr<LowerCamCenterCirclePointDetector> lcCenterCirclePointDetectorSide;
    std::shared_ptr<UpperCamRobotDetector> ucRobotDetector;
    std::shared_ptr<UpperCamDirtyCameraDetector> ucDirtyCameraDetector;
    std::shared_ptr<LowerCameraScrambledCameraDetector> lcScrambledCameraDetector;

    std::shared_ptr<BallPreClassifierUpperCam> ballDetectorUpperCamPreClassifier;
    std::shared_ptr<BallClassifierUpperCam> ballDetectorUpperCamPostClassifier;
    std::shared_ptr<UpperCamPenaltySpotClassifier> ucPenaltySpotClassifier;
    std::shared_ptr<ObjectDetectorLowCam> objectDetectorLowerCam;

    std::shared_ptr<ImagePreprocessor> lcImagePreprocessor;
    std::shared_ptr<ObjectDetectorLowCamHypGen> lcHypGenBall;
    std::shared_ptr<ObjectDetectorLowCamHypGen> lcHypGenPenaltySpot;

    std::shared_ptr<BallDetector> ballDetector;
    std::shared_ptr<PenaltySpotDetector> penaltySpotDetector;
    std::shared_ptr<JerseyDetection> jerseyDetection;

    HTWKVision(HtwkVisionConfig &cfg, ThreadPool* thread_pool);
    HTWKVision(HTWKVision& h) = delete;
    HTWKVision(HTWKVision&& h) = delete;
    HTWKVision& operator=(const HTWKVision&) = delete;
    HTWKVision& operator=(HTWKVision&&) = delete;
    ~HTWKVision();

    void proceed(uint8_t* img, CamPose& cam_pose, bool ultra_low_latency = false);

    std::optional<ObjectHypothesis> getPenaltySpot() const;
    std::optional<ObjectHypothesis> getBall() const;

    const HtwkVisionConfig& getHtwkVisionConfig() const {
        return config;
    }
    const int8_t* getLutCb() {
        return lutCb;
    }
    const int8_t* getLutCr() {
        return lutCr;
    }

    bool isCameraStuck();
};

}  // namespace htwk

#endif  // HTWK_VISION_H
