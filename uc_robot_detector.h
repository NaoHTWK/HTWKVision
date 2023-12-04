#pragma once

#include <base_detector.h>
#include <bounding_box.h>
#include <htwk_vision_config.h>
#include <image_preprocessor.h>
#include <point_2d.h>
#include <tfliteexecuter.h>

#include <memory>

namespace htwk {

struct RobotBoundingBox {
    BoundingBox bb;
    float dist;
    float angle;
    float own_team_prob = 0.5f;

    RobotBoundingBox(const BoundingBox& bb, float dist, float angle) : bb(bb), dist(dist), angle(angle) {}
};

class UpperCamRobotDetector : public BaseDetector {
public:
    UpperCamRobotDetector(int8_t* lutCb, int8_t* lutCr, HtwkVisionConfig &config);
    UpperCamRobotDetector(const UpperCamRobotDetector&) = delete;
    UpperCamRobotDetector(const UpperCamRobotDetector&&) = delete;
    UpperCamRobotDetector& operator=(const UpperCamRobotDetector&) = delete;
    UpperCamRobotDetector& operator=(UpperCamRobotDetector&&) = delete;
    ~UpperCamRobotDetector();

    void proceed(std::shared_ptr<ImagePreprocessor> imagePreprocessor);

    const std::vector<RobotBoundingBox>& getBoundingBoxes() {
        return bounding_boxes_after_nms;
    }

    std::vector<RobotBoundingBox>& getMutableBoundingBoxes() {
        return bounding_boxes_after_nms;
    }

    void drawInputParameter(uint8_t* yuvImage);

private:
    TFLiteExecuter tflite;
    float* input = nullptr;

    static constexpr int input_width = 80;
    static constexpr int input_height = 60;

    static constexpr int output_width = 10;
    static constexpr int output_height = 8;

    static constexpr int channels = 3;
    static constexpr size_t elements_per_anchors_ssd = 5;   // x y w h obj ...
    static constexpr size_t elements_per_anchors_meta = 3;  // dist, sin(a), cos(a)

    static constexpr float object_threshold = 0.6f;

    /*
     * Anchors as defined in python in y, x (which is wrong ...)
     */
    std::vector<std::vector<float>> anchors = {
            {56 / 480.f, 34 / 640.f}, {95 / 480.f, 55 / 640.f}, {173 / 480.f, 98 / 640.f}, {440 / 480.f, 281 / 640.f}};

    std::vector<RobotBoundingBox> bounding_boxes;
    std::vector<RobotBoundingBox> bounding_boxes_after_nms;

    void convertAnchorsToGlobalBoxes(const float* res);
    void nonMaximumSupression();
};

}  // namespace htwk
