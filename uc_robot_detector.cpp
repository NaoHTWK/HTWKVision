#include "uc_robot_detector.h"

#include <algorithm_ext.h>
#include <easy/profiler.h>
#include <emmintrin.h>
#include <stl_ext.h>

#include <cmath>
#include <cstring>
#include <iostream>

namespace htwk {

UpperCamRobotDetector::UpperCamRobotDetector(int8_t* lutCb, int8_t* lutCr, HtwkVisionConfig& config)
    : BaseDetector(lutCb, lutCr, config) {

    tflite.loadModelFromFile(config.tflitePath + "/uc-robot-detector.tflite", {1, input_height, input_width, channels});
    input = tflite.getInputTensor();

    memset(input, 0, input_width * input_height * channels);
}

UpperCamRobotDetector::~UpperCamRobotDetector() {}

void UpperCamRobotDetector::proceed(std::shared_ptr<ImagePreprocessor> imagePreprocessor) {
    EASY_FUNCTION();

    bounding_boxes.clear();
    bounding_boxes_after_nms.clear();

    memcpy(input, imagePreprocessor->getScaledImage().data(), sizeof(*input) * channels * input_width * input_height);

    EASY_BLOCK("TFLite SSD");
    tflite.execute();
    EASY_END_BLOCK;

    EASY_BLOCK("TFLite SSD Postprocessing");
    convertAnchorsToGlobalBoxes(tflite.getOutputTensor());
    nonMaximumSupression();
    EASY_END_BLOCK;
}

void UpperCamRobotDetector::convertAnchorsToGlobalBoxes(const float* res) {
    EASY_FUNCTION();
    const auto sigmoid = [](float v) { return 1.f / (1.f + std::exp(-v)); };

    for (size_t y = 0; y < output_height; y++) {
        for (size_t x = 0; x < output_width; x++) {
            float offset_start = y * (output_width * anchors.size() * (elements_per_anchors_ssd + elements_per_anchors_meta)) +
                                 x * anchors.size() * (elements_per_anchors_ssd + elements_per_anchors_meta);
            for (size_t a = 0; a < anchors.size(); a++) {
                size_t offset = offset_start + a * elements_per_anchors_ssd;
                size_t offset_meta = offset_start + anchors.size() * elements_per_anchors_ssd + a * elements_per_anchors_meta;

                const float* b = res + offset;
                const float* meta = res + offset_meta;

                float objectiness = sigmoid(*(b + 4));

                if (objectiness < object_threshold)
                    continue;

                float bx = (sigmoid(*(b + 0)) + x) / output_width;
                float by = (sigmoid(*(b + 1)) + y) / output_height;
                float bw = std::exp(*(b + 2)) * anchors[a][0];
                float bh = std::exp(*(b + 3)) * anchors[a][1];

                float x1 = clamp((bx - bw / 2) * width, 0.f, (float)width);
                float y1 = clamp((by - bh / 2) * height, 0.f, (float)height);
                float x2 = clamp((bx + bw / 2) * width, 0.f, (float)width);
                float y2 = clamp((by + bh / 2) * height, 0.f, (float)height);

                float dist = sigmoid(meta[0]) * 9.f;
                float angle_sin = tanh(meta[1]);
                float angle_cos = tanh(meta[2]);
                float angle = atan2f(angle_sin, angle_cos);

                BoundingBox bb(point_2d{x1, y1}, point_2d{x2, y2}, objectiness);
                bounding_boxes.emplace_back(bb, dist, angle);
            }
        }
    }
}

void UpperCamRobotDetector::nonMaximumSupression() {
    std::vector<RobotBoundingBox> bb_candidates = bounding_boxes;

    sort(bb_candidates, [](const auto& a, const auto& b) { return a.bb.prob > b.bb.prob; });
    while (!bb_candidates.empty()) {
        RobotBoundingBox obj = bb_candidates.front();
        bounding_boxes_after_nms.push_back(obj);

        erase_if(bb_candidates, [&obj](const auto& rbb) { return obj.bb.iou(rbb.bb) > 0.25f; });
    }
}

void UpperCamRobotDetector::drawInputParameter(uint8_t* yuvImage) {
    const int blockSizeWidth = width / input_width;
    const int blockSizeHeight = height / input_height;

    for (int ySample = 0; ySample < input_height; ySample++) {
        const int startY = ySample * blockSizeHeight;

        for (int xSample = 0; xSample < input_width; xSample++) {
            const int startX = xSample * blockSizeWidth;

            const int vcy = (int)((input[0 + channels * xSample + ySample * channels * input_width]) * 255.f);
            const int vcb = (int)((input[1 + channels * xSample + ySample * channels * input_width]) * 255.f);
            const int vcr = (int)((input[2 + channels * xSample + ySample * channels * input_width]) * 255.f);
            const uint8_t cy = static_cast<uint8_t>(clamp(vcy, 0, 255));
            const uint8_t cb = static_cast<uint8_t>(clamp(vcb, 0, 255));
            const uint8_t cr = static_cast<uint8_t>(clamp(vcr, 0, 255));

            for (int y = startY; y < startY + blockSizeHeight; y++) {
                for (int x = startX; x < startX + blockSizeWidth; x++) {
                    setYCbCr(yuvImage, x, y, cy, cb, cr);
                }
            }
        }
    }
}

}  // namespace htwk
