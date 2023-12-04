#include "htwk_vision.h"

#include <easy/profiler.h>
#include <hypotheses_generator_blur.h>

namespace htwk {

HTWKVision::HTWKVision(HtwkVisionConfig& cfg, ThreadPool* thread_pool)
    : config(cfg), thread_pool(thread_pool) {
    createAdressLookups();
    fieldColorDetector = new FieldColorDetector(lutCb, lutCr, config);
    fieldBorderDetector = std::make_shared<FieldBorderDetector>(lutCb, lutCr, config);
    regionClassifier = new RegionClassifier(lutCb, lutCr, config);
    lineDetector = new LineDetector(lutCb, lutCr, config);
    ballFeatureExtractor = new BallFeatureExtractor(lutCb, lutCr, config);
    ellipseFitter = new RansacEllipseFitter(lutCb, lutCr, config);
    integralImage = new IntegralImage(lutCb, lutCr, config);
    hypothesesGenerator = new HypothesesGeneratorBlur(integralImage, lutCb, lutCr, config);
    obstacleDetectionLowCam = new LowerCamObstacleDetection(lutCb, lutCr, config);

    ucBallHypImagePreprocessor =
            std::make_shared<ImagePreprocessor>(lutCb, lutCr, config, config.ucBallHypGeneratorConfig.scaledImageWidth,
                                                config.ucBallHypGeneratorConfig.scaledImageHeight);
    ucBallHypGenerator = std::make_shared<UpperCamBallHypothesesGenerator>(lutCb, lutCr, config, thread_pool);

    ballDetectorUpperCamPreClassifier =
            std::make_shared<BallPreClassifierUpperCam>(lutCb, lutCr, ballFeatureExtractor, config);
    ballDetectorUpperCamPostClassifier = std::make_shared<BallClassifierUpperCam>(lutCb, lutCr, config);

    ucImagePreprocessor =
            std::make_shared<ImagePreprocessor>(lutCb, lutCr, config, config.ucGoalPostDetectorConfig.scaledImageWidth,
                                                config.ucGoalPostDetectorConfig.scaledImageHeight);
    ucGoalPostDetector = std::make_shared<UpperCamGoalPostDetector>(lutCb, lutCr, config);
    ucCenterCirclePointDetector = std::make_shared<UpperCamCenterCirclePointDetector>(lutCb, lutCr, config);

    ucPenaltySpotClassifier =
            std::make_shared<UpperCamPenaltySpotClassifier>(lutCb, lutCr, ballFeatureExtractor, config);
    objectDetectorLowerCam = std::make_shared<ObjectDetectorLowCam>(lutCb, lutCr, config);

    lcImagePreprocessor =
            std::make_shared<ImagePreprocessor>(lutCb, lutCr, config, config.lcObjectDetectorConfig.scaledImageWidth,
                                                config.lcObjectDetectorConfig.scaledImageHeight);
    lcHypGenBall = std::make_shared<ObjectDetectorLowCamHypGen>(
            lutCb, lutCr, config, config.lcObjectDetectorConfig.hypGenModelBall, lcImagePreprocessor);
    lcHypGenPenaltySpot = std::make_shared<ObjectDetectorLowCamHypGen>(
            lutCb, lutCr, config, config.lcObjectDetectorConfig.hypGenModelPenatlySpot, lcImagePreprocessor);
    lcCenterCirclePointDetectorCenter = std::make_shared<LowerCamCenterCirclePointDetector>(
            lutCb, lutCr, config, LowerCamCenterCirclePointDetector::CENTER);
    lcCenterCirclePointDetectorSide = std::make_shared<LowerCamCenterCirclePointDetector>(
            lutCb, lutCr, config, LowerCamCenterCirclePointDetector::SIDE);
    ucRobotDetector = std::make_shared<UpperCamRobotDetector>(lutCb, lutCr, config);
    ucDirtyCameraDetector = std::make_shared<UpperCamDirtyCameraDetector>(lutCb, lutCr, config, fieldColorDetector);
    lcScrambledCameraDetector = std::make_shared<LowerCameraScrambledCameraDetector>(lutCb, lutCr, config);

    if (config.isUpperCam) {
        ballDetector = ballDetectorUpperCamPostClassifier;
        penaltySpotDetector =
                std::make_shared<PenaltySpotDetectorAdapter<UpperCamPenaltySpotClassifier>>(ucPenaltySpotClassifier);
        jerseyDetection =
                std::make_shared<JerseyDetection>(lutCb, lutCr, config, fieldColorDetector, ucRobotDetector.get());
    } else {
        ballDetector = objectDetectorLowerCam;
        penaltySpotDetector =
                std::make_shared<PenaltySpotDetectorAdapter<ObjectDetectorLowCam>>(objectDetectorLowerCam);
    }
}

HTWKVision::~HTWKVision() {
    delete fieldColorDetector;
    delete regionClassifier;
    delete lineDetector;
    delete ballFeatureExtractor;
    delete ellipseFitter;
    delete integralImage;
    delete hypothesesGenerator;
    delete obstacleDetectionLowCam;

    free(lutCb);
    free(lutCr);
}

void HTWKVision::proceed(uint8_t* img, CamPose& cam_pose, bool ultra_low_latency) {
    EASY_FUNCTION(profiler::colors::Blue);
    TaskScheduler scheduler(thread_pool);

    auto fieldBorder = scheduler.addTask([&]() { fieldBorderDetector->proceed(img); }, {});
    if (!ultra_low_latency) {
        auto regions = scheduler.addTask(
                [&]() {
                    fieldColorDetector->proceed(img);
                    regionClassifier->proceed(img, fieldColorDetector);
                },
                {});
        auto lines = scheduler.addTask(
                [&]() {
                    // LineDetector modifies the LineSegments from RegionClassifier.
                    lineDetector->proceed(
                            img, regionClassifier->getLineSegments(fieldBorderDetector->getConvexFieldBorder()),
                            regionClassifier->lineSpacing);
                },
                {regions, fieldBorder});
        if (config.isUpperCam) {
            scheduler.addTask(
                    [&]() {
                        ellipseFitter->proceed(
                                regionClassifier->getLineSegments(fieldBorderDetector->getConvexFieldBorder()), img);
                    },
                    {regions, fieldBorder, lines});
        }
    }
    if (config.isUpperCam) {
        if (!ultra_low_latency) {
            auto ucImgPrepTask = scheduler.addTask([&]() { ucImagePreprocessor->proceed(img); }, {});
            scheduler.addTask([&]() { ucGoalPostDetector->proceed(cam_pose, ucImagePreprocessor); }, {ucImgPrepTask});
            scheduler.addTask([&]() { ucCenterCirclePointDetector->proceed(cam_pose, ucImagePreprocessor); },
                              {ucImgPrepTask});

            if (!config.onlyLocalization) {
                auto robots =
                        scheduler.addTask([&]() { ucRobotDetector->proceed(ucImagePreprocessor); }, {ucImgPrepTask});
                scheduler.addTask([&]() { jerseyDetection->proceed(img); }, {robots});
            }
        }

        if (!config.onlyLocalization) {
            auto ballHypImgPrep = scheduler.addTask([&]() { ucBallHypImagePreprocessor->proceed(img); }, {});
            auto ballHypGen = scheduler.addTask(
                    [&]() { ucBallHypGenerator->proceed(cam_pose, ucBallHypImagePreprocessor); }, {ballHypImgPrep});
            auto integral = scheduler.addTask([&]() { integralImage->proceed(img); }, {});
            scheduler.addTask([&]() { ucDirtyCameraDetector->proceed(img, ucBallHypImagePreprocessor); },
                              {ballHypImgPrep});

            auto hypos = scheduler.addTask(
                    [&]() {
                        hypothesesGenerator->proceed(img, fieldBorderDetector->getConvexFieldBorder(), cam_pose,
                                                     integralImage);
                    },
                    {integral, fieldBorder});

            if (!ultra_low_latency) {
                scheduler.addTask(
                        [&]() {
                            auto hypotheses = hypothesesGenerator->getHypotheses();
                            ucPenaltySpotClassifier->proceed(img, hypotheses);
                        },
                        {hypos});
            }

            scheduler.addTask(
                    [&]() {
                        auto hypotheses = hypothesesGenerator->getHypotheses();
                        auto new_hypotheses = ucBallHypGenerator->getHypotheses();

                        hypotheses.insert(hypotheses.end(), std::make_move_iterator(new_hypotheses.begin()),
                                          std::make_move_iterator(new_hypotheses.end()));
                        ballDetectorUpperCamPreClassifier->proceed(img, fieldBorderDetector, hypotheses);

                        auto hypothesesCpy = ballDetectorUpperCamPreClassifier->getAllHypothesesWithProb();
                        ballDetectorUpperCamPostClassifier->proceed(img, hypothesesCpy, cam_pose);
                    },
                    {hypos, ballHypGen, fieldBorder});
        }
    } else {
        if (!config.onlyLocalization) {
            scheduler.addTask([&]() { obstacleDetectionLowCam->proceed(cam_pose.head_angles.yaw, img); }, {});

            auto imgPrep = scheduler.addTask([&]() { lcImagePreprocessor->proceed(img); }, {});
            scheduler.addTask([&]() { lcCenterCirclePointDetectorCenter->proceed(cam_pose, lcImagePreprocessor); },
                              {imgPrep});
            scheduler.addTask([&]() { lcCenterCirclePointDetectorSide->proceed(cam_pose, lcImagePreprocessor); },
                              {imgPrep});
            scheduler.addTask([&]() { lcScrambledCameraDetector->proceed(lcImagePreprocessor); },
                              {imgPrep});
            auto hypGenBall = scheduler.addTask([&]() { lcHypGenBall->proceed(cam_pose); }, {imgPrep});
            auto hypGenPenS = scheduler.addTask([&]() { lcHypGenPenaltySpot->proceed(cam_pose); }, {imgPrep});
            scheduler.addTask(
                    [&]() {
                        objectDetectorLowerCam->proceed(img, cam_pose, lcHypGenBall->getObjectHypotheses(),
                                                        lcHypGenPenaltySpot->getObjectHypotheses());
                    },
                    {hypGenBall, hypGenPenS});
        }
    }
    scheduler.run();
}

std::optional<ObjectHypothesis> HTWKVision::getPenaltySpot() const {
    return penaltySpotDetector->getPenaltySpot();
}

std::optional<ObjectHypothesis> HTWKVision::getBall() const {
    return ballDetector->getBall();
}

bool HTWKVision::isCameraStuck() {
    EASY_FUNCTION();
    const auto& ref_data = (config.isUpperCam ? ucImagePreprocessor : lcImagePreprocessor)->getScaledImage();

    if (stuckCameraReferenceImage.empty()) {
        stuckCameraReferenceImage = ref_data;
        return false;
    }

    float sum = 0.f;
    for (size_t i = 0; i < stuckCameraReferenceImage.size(); i++) {
        sum += std::abs(ref_data[i] - stuckCameraReferenceImage[i]);
        stuckCameraReferenceImage[i] = ref_data[i];
    }

    float res = sum / stuckCameraReferenceImage.size();
    return res < 0.00015;
}

/**
 * creates a lookup table for fast access to the yuv422 pixel form in the camera image
 * just use the markos getY(), getCb() and getCr() to get the pixel value from a given coordinate
 *
 * Y0 Cb0  Y1 Cr0  Y2 Cb1  Y3 Cr1 Y4 Cb2 Y5 Cr2 Y6 Cb3 Y7 Cr3
 *  0   1   2   3   4   5   6   7  8   9 10  11 12  13 14  15
 *
 * (Y0,Cb0,Cr0), (Y1,Cb1,Cr1), (Y2,Cb2,Cr1), (Y3,Cb2,Cr2), (Y4,Cb3,Cr2), (Y5,Cb3,Cr3)
 * 0 -> ( 0, 1, 3),
 * 1 -> ( 2, 5, 7),
 * 3 -> ( 6, 9,11),
 * 4 -> ( 8,13,15).
 * 5 -> (10,13,15),
 * 6 -> (12,17,15),
 * 7 -> (14,17,19)

 * 0 -> ( 0, 1, 2),
 * 2 -> ( 4, 9, 7),
 * 4 -> ( 8,13,15).
 * 6 -> (12,17,15),
 */
void HTWKVision::createAdressLookups() {
    lutCb = (int8_t*)malloc(sizeof(*lutCb) * config.width);
    lutCr = (int8_t*)malloc(sizeof(*lutCr) * config.width);
    for (int i = 0; i < config.width; i++) {
        if ((i & 1) == 0) {
            lutCb[i] = 1;
            lutCr[i] = 3;
        } else {
            lutCb[i] = -1;
            lutCr[i] = 1;
        }
    }
}

}  // namespace htwk
