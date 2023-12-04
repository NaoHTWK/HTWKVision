#include "jersey_detection.h"

#include <RoboCupGameControlData.h>

namespace htwk {
using namespace std;

void JerseyDetection::proceed(uint8_t* img) {
    this->img = img;
    vector<RobotBoundingBox>& robots = uc_robot_detector->getMutableBoundingBoxes();
    params = prepareDetectionBoxes(robots);
    body_color = getBodyColor(params);
    estimateColorProbabilities(params, body_color, robots);
}

vector<JerseyDetectionParams> JerseyDetection::prepareDetectionBoxes(const vector<RobotBoundingBox>& robots) {
    vector<JerseyDetectionParams> params;
    for (const RobotBoundingBox& r : robots) {
        BoundingBox bb = r.bb;
        if (bb.a.x < 5) {
            int new_width = avg_box_ratio * (bb.b.y - bb.a.y);
            if (new_width > bb.b.x - bb.a.x)
                bb.a.x = bb.b.x - new_width;
        } else if (bb.b.x >= width - 5) {
            int new_width = avg_box_ratio * (bb.b.y - bb.a.y);
            if (new_width > bb.b.x - bb.a.x)
                bb.b.x = bb.a.x + new_width;
        }
        params.push_back({.box = bb,
                          .p = {(bb.a.x + bb.b.x) / 2.f, bb.a.y + (bb.b.y - bb.a.y) * jersey_height_rel},
                          .diameter = (int)((bb.b.x - bb.a.x) * jersey_diameter_rel)});
    }
    return params;
}

color JerseyDetection::getBodyColor(const vector<JerseyDetectionParams>& params) {
    color avg_white(0, 0, 0);
    int cnt_white = 0;
    for (const JerseyDetectionParams& p : params) {
        BoundingBox r = p.box;
        int step_width = max(1.f, sqrt(p.box.area()) / 16);
        for (int y = r.center().y; y < r.b.y; y += step_width) {
            for (int x = r.a.x; x < r.b.x; x += step_width) {
                if (x < 0 || x >= width || y < 0 || y >= height)
                    continue;
                point_2d d = point_2d(x, y) - p.p;
                if (d.norm() <= p.diameter / 2)
                    continue;
                color c = getColor(img, x, y);
                if (!field_color_detector->maybeGreen(c)) {
                    avg_white += c;
                    cnt_white++;
                }
            }
        }
    }
    if (cnt_white > 0)
        avg_white /= cnt_white;
    return avg_white;
}

void JerseyDetection::estimateColorProbabilities(const vector<JerseyDetectionParams>& params, const color& body_color,
                                                 vector<RobotBoundingBox>& robots) {
    color green = field_color_detector->getColor();
    color own = body_color + own_color_rel;
    color other = body_color + opp_color_rel;
    zip(robots, params, [&](RobotBoundingBox& robot, const JerseyDetectionParams& params) {
        int cnt_own = 0;
        int cnt_other = 0;
        int cnt = 0;
        int step_width = max(1, params.diameter / 16);
        for (int y = params.p.y - params.diameter / 2; y < params.p.y + params.diameter / 2; y += step_width) {
            for (int x = params.p.x - params.diameter / 2; x < params.p.x + params.diameter / 2; x += step_width) {
                if (x < 0 || x >= width || y < 0 || y >= height)
                    continue;
                point_2d d = point_2d(x, y) - params.p;
                if (d.norm() > params.diameter / 2)
                    continue;
                color c = getColor(img, x, y);
                float dist_green = c.dist(green, 4);
                float dist_own = c.dist(own, 4);
                float dist_other = c.dist(other, 4);
                float dist_body = c.dist(body_color, 4);
                if (dist_own < min(dist_green, min(dist_other, dist_body))) {
                    cnt_own++;
                }
                if (dist_other < min(dist_own, min(dist_green, dist_body))) {
                    cnt_other++;
                }
                cnt++;
            }
        }
        if (cnt == 0)
            return;
        float p_own = cnt_own / (float)cnt;
        float p_other = cnt_other / (float)cnt;
        float p = 0.5f + (p_own * 0.5f - p_other * 0.5f);
        robot.own_team_prob = p;
    });
}

void JerseyDetection::drawDebugOutput(uint8_t* img) {
    color green = field_color_detector->getColor();
    color own = body_color + own_color_rel;
    color other = body_color + opp_color_rel;
    for (const JerseyDetectionParams& p : params) {
        BoundingBox r = p.box;
        for (int y = r.center().y; y < r.b.y; y++) {
            for (int x = r.a.x; x < r.b.x; x++) {
                if (x < 0 || x >= width || y < 0 || y >= height)
                    continue;
                point_2d d = point_2d(x, y) - p.p;
                if (d.norm() <= p.diameter / 2)
                    continue;
                color c = getColor(img, x, y);
                if (!field_color_detector->maybeGreen(c)) {
                    setColor(img, x, y, body_color);
                }
            }
        }
        int step_width = max(1, p.diameter / 16);
        for (int y = p.p.y - p.diameter / 2; y < p.p.y + p.diameter / 2; y += step_width) {
            for (int x = p.p.x - p.diameter / 2; x < p.p.x + p.diameter / 2; x += step_width) {
                if (x < 0 || x >= width || y < 0 || y >= height)
                    continue;
                point_2d d = point_2d(x, y) - p.p;
                if (d.norm() > p.diameter / 2)
                    continue;
                color c = getColor(img, x, y);
                float dist_green = c.dist(green, 4);
                float dist_own = c.dist(own, 4);
                float dist_other = c.dist(other, 4);
                float dist_body = c.dist(body_color, 4);
                if (dist_own < min(dist_green, min(dist_other, dist_body))) {
                    setYCbCr(img, x, y, 80, 255, 128);
                }
                if (dist_other < min(dist_own, min(dist_green, dist_body))) {
                    setYCbCr(img, x, y, 160, 0, 128);
                }
            }
        }
    }
}

void JerseyDetection::teamColorCallback(uint8_t own_team_id, uint8_t own_team_color, uint8_t opp_team_id,
                                        uint8_t opp_team_color) {
    switch (own_team_color) {
        case TEAM_BLUE:
            own_color_rel = {-50, 20, -10};
            break;
        case TEAM_YELLOW:
            own_color_rel = {-40, -20, 5};
            break;
        default:
            printf("!!!!!!!!!!!! Unknown own team color !!!!!!!!!!!!!!!!!\n");
    }
    switch (opp_team_color) {
        case TEAM_BLUE:
            opp_color_rel = {-50, 20, -10};
            break;
        case TEAM_YELLOW:
            opp_color_rel = {-40, -20, 5};
            break;
        case TEAM_RED:
            opp_color_rel = {-40, -10, 40};
            break;
        case TEAM_BLACK:
        case TEAM_GRAY:
            opp_color_rel = {-80, 0, 0};
            break;
        default:
            printf("!!!!!!!!!!!! Unknown opp team color !!!!!!!!!!!!!!!!!\n");
    }
}

}  // namespace htwk
