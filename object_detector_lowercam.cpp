#include "object_detector_lowercam.h"

#include <cstring>

#include <easy/profiler.h>
#include <emmintrin.h>

#include <algorithm_ext.h>
#include <line.h>
#include <stl_ext.h>

using namespace std;

namespace htwk {

ObjectDetectorLowCam::ObjectDetectorLowCam(int8_t* lutCb, int8_t* lutCr, HtwkVisionConfig &config)
    : BallDetector(lutCb, lutCr, config),
      inputWidth(config.lcObjectDetectorConfig.scaledImageWidth),
      inputHeight(config.lcObjectDetectorConfig.scaledImageHeight),
      hypo_size_x(config.lcObjectDetectorConfig.patchSize),
      hypo_size_y(config.lcObjectDetectorConfig.patchSize)
{
    ballHypothesis.resize(1);
    penaltySpotHypothesis.resize(1);

    if(config.lcObjectDetectorConfig.classifyObjData) {
        hypClassifierExecuter.loadModelFromFile(config.tflitePath + "/lc-object-classifier.tflite", {2, hypo_size_y, hypo_size_x, channels});
        inputHypClassifier = hypClassifierExecuter.getInputTensor();
    } else {
        size_t alloc_size = 2*hypo_size_y*hypo_size_x*channels;
        // aligned_alloc expects multiple of the alignment size as size.
        inputHypClassifier = (float*)aligned_alloc(16, ((alloc_size * sizeof(float) + 15) / 16) * 16);

        if (inputHypClassifier == nullptr) {
            fprintf(stdout, "%s:%d: %s error allocation input array!", __FILE__, __LINE__, __func__);
            exit(1);
        }
    }
}

ObjectDetectorLowCam::~ObjectDetectorLowCam() {
    if(!config.lcObjectDetectorConfig.classifyObjData) {
        free(inputHypClassifier);
    }
}

void ObjectDetectorLowCam::proceed(uint8_t* img, CamPose& cam_pose, ObjectHypothesis inBallHyp, ObjectHypothesis inPenaltySpotHyp) {
    Timer t("ObjectDetectorLowCam", 50);
    EASY_FUNCTION();

    auto& ballHyp = ballHypothesis[0];
    auto& penaltySpotHyp = penaltySpotHypothesis[0];

    ballHyp = inBallHyp;
    penaltySpotHyp = inPenaltySpotHyp;

    EASY_BLOCK("ObjectDetectorLowerCam Gen Hyp");
    generateHypothesis(img, cam_pose, ballHyp, inputHypClassifier);
    generateHypothesis(img, cam_pose, penaltySpotHyp, inputHypClassifier + hypo_size_x*hypo_size_y*channels);
    EASY_END_BLOCK;

    if(!config.lcObjectDetectorConfig.classifyObjData)
        return;

    EASY_BLOCK("ObjectDetectorLowerCam Classifier");
    hypClassifierExecuter.execute();
    EASY_END_BLOCK;

    ballHyp.prob = hypClassifierExecuter.getOutputTensor()[1];
    if (ballHyp.prob >= config.lcObjectDetectorConfig.ballProbabilityThreshold) {
        ballClassifierResult = ballHyp;
    } else {
        ballClassifierResult = std::nullopt;
    }
    //printf("Ball prob: %.2f\n", ballHyp.prob);

    penaltySpotHyp.prob = hypClassifierExecuter.getOutputTensor()[5];
    if (penaltySpotHyp.prob >= config.lcObjectDetectorConfig.penaltySpotProbabilityThreshold) {
        penaltySpotClassifierResult = penaltySpotHyp;
    } else {
        penaltySpotClassifierResult = std::nullopt;
    }
    //printf("PenaltySpot prob: %.2f\n", penaltySpotHyp.prob);

}

void ObjectDetectorLowCam::generateHypothesis(uint8_t* img, CamPose& cam_pose, const ObjectHypothesis& hypPos, float* output) {
    EASY_FUNCTION();
    if (auto radius = LocalizationUtils::getPixelRadius(hypPos, cam_pose, 0.05f + 0.025f)) {
        for (int hy = 0; hy < hypo_size_y; hy++) {
            for (int hx = 0; hx < hypo_size_x; hx++) {
                int y = 0;
                int u = 0;
                int v = 0;
                int cnt = 0;
                for (int img_y = round_int(hypPos.y - *radius + hy * *radius / (hypo_size_y / 2));
                     img_y < round_int(hypPos.y - *radius + (hy + 1) * *radius / (hypo_size_y / 2)); img_y++) {
                    for (int img_x = round_int(hypPos.x - *radius + hx * *radius / (hypo_size_x / 2));
                         img_x < round_int(hypPos.x - *radius + (hx + 1) * *radius / (hypo_size_x / 2)); img_x++) {

                        int32_t tmp_img_x = clamp(img_x, 0, width - 1);
                        int32_t tmp_img_y = clamp(img_y, 0, height - 1);
                        y += getY(img, tmp_img_x, tmp_img_y);
                        u += getCb(img, tmp_img_x, tmp_img_y);
                        v += getCr(img, tmp_img_x, tmp_img_y);
                        cnt++;
                    }
                }
                if (cnt == 0) {
                    int img_x = clamp(round_int(hypPos.x - *radius + (hx + 0.5f) * *radius / (hypo_size_x / 2)), 0, width - 1);
                    int img_y = clamp(round_int(hypPos.y - *radius + (hy + 0.5f) * *radius / (hypo_size_y / 2)), 0, height - 1);
                    if (img_x < 0 || img_x >= width || img_y < 0 || img_y >= height) {
                        output[0 + hx * 3 + hy * 3 * hypo_size_x] = 0.f;
                        output[1 + hx * 3 + hy * 3 * hypo_size_x] = 0.f;
                        output[2 + hx * 3 + hy * 3 * hypo_size_x] = 0.f;
                    } else {
                        output[0 + hx * 3 + hy * 3 * hypo_size_x] = getY(img, img_x, img_y) / 255.f;
                        output[1 + hx * 3 + hy * 3 * hypo_size_x] = getCb(img, img_x, img_y) / 255.f;
                        output[2 + hx * 3 + hy * 3 * hypo_size_x] = getCr(img, img_x, img_y) / 255.f;
                    }
                } else {
                    output[0 + hx * 3 + hy * 3 * hypo_size_x] = y / (255.f * cnt);
                    output[1 + hx * 3 + hy * 3 * hypo_size_x] = u / (255.f * cnt);
                    output[2 + hx * 3 + hy * 3 * hypo_size_x] = v / (255.f * cnt);
                }
            }
        }
    }
}

std::vector<float> ObjectDetectorLowCam::generateAugmentedHypothesis(uint8_t* img, CamPose& cam_pose, const ObjectHypothesis& hypPos, Augmentation aug) {
    std::vector<float> res(hypo_size_x * hypo_size_y * 3, 0.f);
    if (auto opt_radius = LocalizationUtils::getPixelRadius(hypPos, cam_pose, (0.05f + 0.025f) * aug.scale)) {
        float radius = *opt_radius;
        point_2d hyp = hypPos + aug.translation * radius * 2 * aug.scale;
        float pixel_size_x = radius / (hypo_size_x / 2);
        float pixel_size_y = radius / (hypo_size_y / 2);
        for (int hy = 0; hy < hypo_size_y; hy++) {
            for (int hx = 0; hx < hypo_size_x; hx++) {
                int y = 0;
                int u = 0;
                int v = 0;
                int cnt = 0;
                vector<point_2d> points{{-radius + hx * pixel_size_x, -radius + hy * pixel_size_y},
                                        {-radius + (hx + 1) * pixel_size_x, -radius + hy * pixel_size_y},
                                        {-radius + (hx + 1) * pixel_size_x, -radius + (hy + 1) * pixel_size_y},
                                        {-radius + hx * pixel_size_x, -radius + (hy + 1) * pixel_size_y}};
                for (point_2d& p : points)
                    p = p.rotated(aug.rotation) + hyp;
                vector<Line> lines{Line(points[0], points[1]), Line(points[1], points[2]), Line(points[2], points[3]),
                                   Line(points[3], points[0])};
                int min_x = floor(min(min(points[0].x, points[1].x), min(points[2].x, points[3].x)));
                int min_y = floor(min(min(points[0].y, points[1].y), min(points[2].y, points[3].y)));
                int max_x = ceil(max(max(points[0].x, points[1].x), max(points[2].x, points[3].x)));
                int max_y = ceil(max(max(points[0].y, points[1].y), max(points[2].y, points[3].y)));
                for (int img_y = min_y; img_y <= max_y; img_y++) {
                    for (int img_x = min_x; img_x < max_x; img_x++) {
                        bool inside = true;
                        for (int i = 0; i < 4; i++) {
                            // We use another coordinate system here (image vs field for line) so we have to test '<' instead '>'
                            if (lines[i].side({img_x + 0.5f, img_y + 0.5f}) < 0) {
                                inside = false;
                                break;
                            }
                        }
                        if (!inside)
                            continue;

                        int32_t tmp_img_x = clamp(img_x, 0, width - 1);
                        int32_t tmp_img_y = clamp(img_y, 0, height - 1);
                        y += getY(img, tmp_img_x, tmp_img_y);
                        u += getCb(img, tmp_img_x, tmp_img_y);
                        v += getCr(img, tmp_img_x, tmp_img_y);
                        cnt++;
                    }
                }
                if (cnt == 0) {
                    int img_x = clamp(round_int(((points[0] + points[2]) / 2).x), 0, width - 1);
                    int img_y = clamp(round_int(((points[0] + points[2]) / 2).y), 0, height - 1);
                    res[0 + hx * 3 + hy * 3 * hypo_size_x] = getY(img, img_x, img_y) / 255.f;
                    res[1 + hx * 3 + hy * 3 * hypo_size_x] = getCb(img, img_x, img_y) / 255.f;
                    res[2 + hx * 3 + hy * 3 * hypo_size_x] = getCr(img, img_x, img_y) / 255.f;
                } else {
                    res[0 + hx * 3 + hy * 3 * hypo_size_x] = y / (255.f * cnt);
                    res[1 + hx * 3 + hy * 3 * hypo_size_x] = u / (255.f * cnt);
                    res[2 + hx * 3 + hy * 3 * hypo_size_x] = v / (255.f * cnt);
                }
            }
        }
    }
    if (aug.mirror) {
        for (int y = 0; y < hypo_size_y; y++) {
            for (int x = 0; x < hypo_size_x / 2; x++) {
                std::swap(res[0 + x * 3 + y * 3 * hypo_size_x],
                          res[0 + (hypo_size_x - 1 - x) * 3 + y * 3 * hypo_size_x]);
                std::swap(res[1 + x * 3 + y * 3 * hypo_size_x],
                          res[1 + (hypo_size_x - 1 - x) * 3 + y * 3 * hypo_size_x]);
                std::swap(res[2 + x * 3 + y * 3 * hypo_size_x],
                          res[2 + (hypo_size_x - 1 - x) * 3 + y * 3 * hypo_size_x]);
            }
        }
    }
    return res;
}

}  // namespace htwk
