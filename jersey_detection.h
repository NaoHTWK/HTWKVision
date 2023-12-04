#pragma once

#include <base_detector.h>
#include <color.h>
#include <field_color_detector.h>
#include <uc_robot_detector.h>

#include <vector>

namespace htwk {

struct JerseyDetectionParams {
    BoundingBox box;
    point_2d p;
    int diameter;
};

class JerseyDetection : protected BaseDetector {
public:
    JerseyDetection(int8_t* lutCb, int8_t* lutCr, HtwkVisionConfig& config,
                    FieldColorDetector* field_color_detector, UpperCamRobotDetector* uc_robot_detector)
        : BaseDetector(lutCb, lutCr, config),
          field_color_detector(field_color_detector),
          uc_robot_detector(uc_robot_detector) {
    }
    JerseyDetection(const JerseyDetection&) = delete;
    JerseyDetection(JerseyDetection&&) = delete;
    JerseyDetection& operator=(const JerseyDetection&) = delete;
    JerseyDetection& operator=(JerseyDetection&&) = delete;

    void proceed(uint8_t* img) __attribute__((nonnull));
    void drawDebugOutput(uint8_t* img);
    void teamColorCallback(uint8_t own_team_id, uint8_t own_team_color, uint8_t opp_team_id, uint8_t opp_team_color);

private:
    static constexpr float jersey_height_rel = 0.35f;
    static constexpr float jersey_diameter_rel = 0.5f;
    static constexpr float avg_box_ratio = 0.55f;

    // optimize positions of robot detection bounding boxes
    std::vector<JerseyDetectionParams> prepareDetectionBoxes(const std::vector<RobotBoundingBox>& robots);
    // averages color from white body parts. used as color normalization values
    color getBodyColor(const std::vector<JerseyDetectionParams>& params);
    // estimating color probabilities by pixel based color class counting
    void estimateColorProbabilities(const std::vector<JerseyDetectionParams>& params, const color& body_color,
                                    std::vector<RobotBoundingBox>& robots);

    FieldColorDetector* field_color_detector;
    UpperCamRobotDetector* uc_robot_detector;
    color own_color_rel{-50, 20, -10};
    color opp_color_rel{-40, -20, 5};
    uint8_t* img;
    std::vector<JerseyDetectionParams> params;
    color body_color;
};

}  // namespace htwk
