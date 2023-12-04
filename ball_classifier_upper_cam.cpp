#include "ball_classifier_upper_cam.h"

#include <algorithm_ext.h>
#include <easy/profiler.h>
#include <emmintrin.h>
#include <line.h>
#include <stl_ext.h>

#include <cstring>

using namespace std;

namespace htwk {

BallClassifierUpperCam::BallClassifierUpperCam(int8_t* lutCb, int8_t* lutCr, HtwkVisionConfig &config)
    : BallDetector(lutCb, lutCr, config),
      hypo_size_x(config.ucBallLargeClassifierConfig.patchSize),
      hypo_size_y(config.ucBallLargeClassifierConfig.patchSize),
      num_hypotheses_to_test(config.ucBallLargeClassifierConfig.camHypothesesCount),
      smallBallProbThreshold(config.ucBallLargeClassifierConfig.smallBallProbabilityThreshold),
      shouldWeClassifyBallData(config.ucBallLargeClassifierConfig.classifyData) {

    if (shouldWeClassifyBallData) {
        tflite.loadModelFromFile(config.tflitePath + "/uc-ball-large-classifier.tflite",
                                 {(int)num_hypotheses_to_test, hypo_size_y, hypo_size_x, channels});
        classifierInput = tflite.getInputTensor();
    } else {
        classifierInput = (float*)aligned_alloc(
                16, (((num_hypotheses_to_test * channels * hypo_size_x * hypo_size_y * sizeof(float)) + 15) / 16) * 16);

        if (classifierInput == nullptr) {
            printf("Error allocating memory!\n");
            exit(1);
        }
    }

    ratedBallHypothesis.resize(num_hypotheses_to_test);
}

BallClassifierUpperCam::~BallClassifierUpperCam() {
    if (!shouldWeClassifyBallData)
        free(classifierInput);
}

// void BallClassifierUpperCam::drawInputParameter(uint8_t* yuvImage) {
//    const int blockSizeWidth = width / inputWidth;
//    const int blockSizeHeight = height / inputHeight;

//    for (int ySample = 0; ySample < inputHeight; ySample++) {
//        const int startY = ySample * blockSizeHeight;

//        for (int xSample = 0; xSample < inputWidth; xSample++) {
//            const int startX = xSample * blockSizeWidth;

//            const int vcy = (int)((xlaHypGenInput[0 + channels * xSample + ySample * channels * inputWidth] + 1.f) *
//            128.f); const int vcb = (int)((xlaHypGenInput[1 + channels * xSample + ySample * channels * inputWidth]
//            + 1.f) * 128.f); const int vcr = (int)((xlaHypGenInput[2 + channels * xSample + ySample * channels *
//            inputWidth] + 1.f) * 128.f); const uint8_t cy = static_cast<uint8_t>(std::min(std::max(0, vcy), 255));
//            const uint8_t cb = static_cast<uint8_t>(std::min(std::max(0, vcb), 255));
//            const uint8_t cr = static_cast<uint8_t>(std::min(std::max(0, vcr), 255));

//            for (int y = startY; y < startY + blockSizeHeight; y++) {
//                for (int x = startX; x < startX + blockSizeWidth; x++) {
//                    setYCbCr(yuvImage, x, y, cy, cb, cr);
//                }
//            }
//        }
//    }
//}

void BallClassifierUpperCam::proceed(uint8_t* img, std::vector<ObjectHypothesis>& hypotheses, CamPose& cam_pose) {
    Timer t("BallClassifierUpperCam", 50);
    EASY_FUNCTION();

    std::sort(hypotheses.begin(), hypotheses.end(),
              [](const ObjectHypothesis& a, const ObjectHypothesis& b) { return a.prob > b.prob; });

    ratedBallHypothesis.clear();
    allRatedHypothesis.clear();
    ballClassifierResult = std::nullopt;

    // Do nothing if we are not very sure that there is even a ball ...
    if (hypotheses.empty() || hypotheses[0].prob < smallBallProbThreshold)
        return;

    EASY_BLOCK("Hyp Gen");
    size_t src_idx = 0;
    size_t dest_idx = 0;

    while(dest_idx < std::min(num_hypotheses_to_test, hypotheses.size()) && src_idx < hypotheses.size()) {
        ObjectHypothesis hyp = hypotheses[dest_idx];
        auto radius = LocalizationUtils::getPixelRadius(hyp, cam_pose, 0.05f);

        if (!radius) {
            src_idx++;
            continue;
        }

        if(*radius > config.width / 4) {
            printf("BallClassifierUpperCam: getPixelRadius returned an unusal big size of %d\n", round_int(*radius));
            src_idx++;
            continue;
        }

        allRatedHypothesis.push_back(hyp);
        if (shouldWeClassifyBallData)
            generateHypothesis(img, cam_pose, hyp, dest_idx * channels * hypo_size_x * hypo_size_y);

        src_idx++;
        dest_idx++;
    }
    EASY_END_BLOCK;

    if (!shouldWeClassifyBallData)
        return;

    EASY_BLOCK("TFLite BallDetectorUpperCam Classify");
    tflite.execute();
    EASY_END_BLOCK;
    const float* curHypResult = tflite.getOutputTensor();
    float maxBallProb = config.ucBallLargeClassifierConfig.probabilityThreshold;

    for (size_t i = 0; i < std::min(dest_idx, std::min(num_hypotheses_to_test, hypotheses.size())); i++) {
        auto& hyp = allRatedHypothesis[i];  // yes no reference
        allRatedHypothesis[i].prob = curHypResult[2 * i + 1];

        if (hyp.prob >= config.ucBallLargeClassifierConfig.probabilityThreshold) {
            ratedBallHypothesis.push_back(hyp);

            if (hyp.prob > maxBallProb) {
                maxBallProb = hyp.prob;
                ballClassifierResult = hyp;
            }
        }
    }
}

void BallClassifierUpperCam::generateHypothesis(uint8_t* img, CamPose& cam_pose, ObjectHypothesis& hyp, size_t offset) {
    EASY_FUNCTION();
    if (auto radius = LocalizationUtils::getPixelRadius(hyp, cam_pose, 0.05f + 0.025f)) {
        for (int hy = 0; hy < hypo_size_y; hy++) {
            for (int hx = 0; hx < hypo_size_x; hx++) {
                int y = 0;
                int u = 0;
                int v = 0;
                int cnt = 0;
                for (int img_y = round_int(hyp.y - *radius + hy * *radius / (hypo_size_y / 2));
                     img_y < round_int(hyp.y - *radius + (hy + 1) * *radius / (hypo_size_y / 2)); img_y++) {
                    for (int img_x = round_int(hyp.x - *radius + hx * *radius / (hypo_size_x / 2));
                         img_x < round_int(hyp.x - *radius + (hx + 1) * *radius / (hypo_size_x / 2)); img_x++) {

                        int32_t tmp_img_x = clamp(img_x, 0, width - 1);
                        int32_t tmp_img_y = clamp(img_y, 0, height - 1);
                        y += getY(img, tmp_img_x, tmp_img_y);
                        u += getCb(img, tmp_img_x, tmp_img_y);
                        v += getCr(img, tmp_img_x, tmp_img_y);
                        cnt++;
                    }
                }
                if (cnt == 0) {
                    int img_x =
                            clamp(round_int(hyp.x - *radius + (hx + 0.5f) * *radius / (hypo_size_x / 2)), 0, width - 1);
                    int img_y = clamp(round_int(hyp.y - *radius + (hy + 0.5f) * *radius / (hypo_size_y / 2)), 0,
                                      height - 1);
                    if (img_x < 0 || img_x >= width || img_y < 0 || img_y >= height) {
                        classifierInput[offset + 0 + hx * 3 + hy * 3 * hypo_size_x] = 0.f;
                        classifierInput[offset + 1 + hx * 3 + hy * 3 * hypo_size_x] = 0.f;
                        classifierInput[offset + 2 + hx * 3 + hy * 3 * hypo_size_x] = 0.f;
                    } else {
                        classifierInput[offset + 0 + hx * 3 + hy * 3 * hypo_size_x] =
                                getY(img, img_x, img_y) / 128.f - 1.f;
                        classifierInput[offset + 1 + hx * 3 + hy * 3 * hypo_size_x] =
                                getCb(img, img_x, img_y) / 128.f - 1.f;
                        classifierInput[offset + 2 + hx * 3 + hy * 3 * hypo_size_x] =
                                getCr(img, img_x, img_y) / 128.f - 1.f;
                    }
                } else {
                    classifierInput[offset + 0 + hx * 3 + hy * 3 * hypo_size_x] = y / (128.f * cnt) - 1.f;
                    classifierInput[offset + 1 + hx * 3 + hy * 3 * hypo_size_x] = u / (128.f * cnt) - 1.f;
                    classifierInput[offset + 2 + hx * 3 + hy * 3 * hypo_size_x] = v / (128.f * cnt) - 1.f;
                }
            }
        }
    }
}

std::vector<float> BallClassifierUpperCam::generateAugmentedHypothesis(uint8_t* img, const ObjectHypothesis& obj_hyp,
                                                                       CamPose& cam_pose, Augmentation aug) {
    std::vector<float> res(hypo_size_x * hypo_size_y * 3, 0.f);
    if (auto opt_radius = LocalizationUtils::getPixelRadius(obj_hyp, cam_pose, (0.05f + 0.025f) * aug.scale)) {
        float radius = *opt_radius;
        point_2d hyp = obj_hyp + aug.translation * radius * 2 * aug.scale;
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
                            // We use another coordinate system here (image vs field for line) so we have to test '<'
                            // instead '>'
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
                    res[0 + hx * 3 + hy * 3 * hypo_size_x] = getY(img, img_x, img_y) / 128.f - 1.f;
                    res[1 + hx * 3 + hy * 3 * hypo_size_x] = getCb(img, img_x, img_y) / 128.f - 1.f;
                    res[2 + hx * 3 + hy * 3 * hypo_size_x] = getCr(img, img_x, img_y) / 128.f - 1.f;
                } else {
                    res[0 + hx * 3 + hy * 3 * hypo_size_x] = y / (128.f * cnt) - 1.f;
                    res[1 + hx * 3 + hy * 3 * hypo_size_x] = u / (128.f * cnt) - 1.f;
                    res[2 + hx * 3 + hy * 3 * hypo_size_x] = v / (128.f * cnt) - 1.f;
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
