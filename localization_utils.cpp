#include <cam_constants.h>
#include <localization_utils.h>
#include <point_3d.h>

using namespace std;
using namespace htwk;

void testTranslation(const std::optional<point_3d>& translation) {
    if (!translation) {
        printf("%s:%d - %s: Nein! Nein! Nein!", __FILE__, __LINE__, __PRETTY_FUNCTION__);
        fflush(stdout);
        exit(1);
    }
}

optional<point_2d> LocalizationUtils::project(const point_2d& p_, const CamPose& cam_pose) {
    return project(p_, 0, cam_pose);
}

optional<point_2d> LocalizationUtils::project(const point_2d& p_, float height, const CamPose& cam_pose) {
    if (cam_pose.v5_angles)
        return LocalizationUtils::projectV5(p_, cam_pose);

    testTranslation(cam_pose.translation);
    point_3d p(cam_depth, -p_.x + cam_width / 2, -p_.y + cam_height / 2);
    if (cam_pose.ellipse_angles)
        p = p.rotated_y(cam_pose.ellipse_angles->pitch)
                    .rotated_x(cam_pose.ellipse_angles->roll)
                    .rotated_z(cam_pose.head_angles.yaw);
    else
        p = p.rotated_x(cam_pose.cam_id == CamID::UPPER ? cam_pose.head_offset.roll : 0)
                    .rotated_y(cam_pose.head_angles.pitch + (cam_pose.cam_id == CamID::UPPER
                                                                     ? upper_cam_pitch + cam_pose.head_offset.pitch
                                                                     : lower_cam_pitch))
                    .rotated_z(cam_pose.head_angles.yaw)
                    .rotated_y(cam_pose.body_angles.pitch + cam_pose.body_offset.pitch)
                    .rotated_x(cam_pose.body_angles.roll + cam_pose.body_offset.roll);
    if (p.z >= 0)
        return {};
    float n = -(cam_pose.translation.z - height) / p.z;
    p *= n;
    p += cam_pose.translation;
    return point_2d{p.x, p.y};
}

optional<Line> LocalizationUtils::project(const Line& l, const CamPose& cam_pose) {
    if (auto p1 = project(l.p1(), cam_pose)) {
        if (auto p2 = project(l.p2(), cam_pose)) {
            return Line(*p1, *p2);
        }
    }
    return {};
}

point_3d LocalizationUtils::relToCam(point_3d p, const CamPose& cam_pose) {
    testTranslation(cam_pose.translation);
    p -= cam_pose.translation;
    if (cam_pose.ellipse_angles)
        p = p.rotated_z(-cam_pose.head_angles.yaw)
                    .rotated_x(-cam_pose.ellipse_angles->roll)
                    .rotated_y(-cam_pose.ellipse_angles->pitch);
    else
        p = p.rotated_x(-cam_pose.body_angles.roll - cam_pose.body_offset.roll)
                    .rotated_y(-cam_pose.body_angles.pitch - cam_pose.body_offset.pitch)
                    .rotated_z(-cam_pose.head_angles.yaw)
                    .rotated_y(-cam_pose.head_angles.pitch - (cam_pose.cam_id == CamID::UPPER
                                                                      ? upper_cam_pitch + cam_pose.head_offset.pitch
                                                                      : lower_cam_pitch))
                    .rotated_x(cam_pose.cam_id == CamID::UPPER ? -cam_pose.head_offset.roll : 0);
    return p;
}

point_3d LocalizationUtils::relToCam(const point_2d& p, const CamPose& cam_pose) {
    return relToCam({p.x, p.y, 0}, cam_pose);
}

std::optional<point_2d> LocalizationUtils::camToImage(point_3d p) {
    if (p.x <= 0)
        return {};
    p *= cam_depth / p.x;
    return point_2d{-p.y + cam_width / 2, -p.z + cam_height / 2};
}

point_2d LocalizationUtils::relToAbs(const point_2d& p, const Position& pos) {
    return pos.point() + p.rotated(pos.a);
}

point_2d LocalizationUtils::absToRel(const point_2d& p, const Position& pos) {
    return (p - pos.point()).rotated(-pos.a);
}

optional<point_2d> LocalizationUtils::neckToCam(point_3d p, const YawPitch& head_pos, CamID cam_id) {
    point_3d cam_offset_to_neck = (cam_id == CamID::UPPER ? upper_cam_neck : lower_cam_neck)
                                          .rotated_y(head_pos.pitch)
                                          .rotated_z(head_pos.yaw);
    p = p - cam_offset_to_neck;
    p = p.rotated_x(-(cam_id == CamID::UPPER ? CamOffset::headRoll : 0))
                .rotated_z(-head_pos.yaw)
                .rotated_y(-(cam_id == CamID::UPPER ? upper_cam_pitch + CamOffset::headPitch : lower_cam_pitch) -
                           head_pos.pitch);
    // p is behind/inside the camera.
    if (p.x <= 0)
        return {};
    return point_2d(-p.y, -p.z) * (cam_depth / p.x) + point_2d(cam_width / 2, cam_height / 2);
}

optional<Line> LocalizationUtils::getHorizon(const CamPose& cam_pose) {
    if (cam_pose.v5_angles)
        return LocalizationUtils::getHorizonV5(cam_pose);
    // This is by far not the most efficient way to do this but we've got the CPU power. :)
    point_3d p1 = point_3d(1, 0, 0).rotated_z(1_deg + cam_pose.head_angles.yaw);
    point_3d p2 = point_3d(1, 0, 0).rotated_z(-1_deg + cam_pose.head_angles.yaw);
    if (cam_pose.ellipse_angles) {
        p1 = p1.rotated_z(-cam_pose.head_angles.yaw)
                     .rotated_x(-cam_pose.ellipse_angles->roll)
                     .rotated_y(-cam_pose.ellipse_angles->pitch);
        p2 = p2.rotated_z(-cam_pose.head_angles.yaw)
                     .rotated_x(-cam_pose.ellipse_angles->roll)
                     .rotated_y(-cam_pose.ellipse_angles->pitch);
    } else {
        p1 = p1.rotated_x(-cam_pose.body_angles.roll - cam_pose.body_offset.roll)
                     .rotated_y(-cam_pose.body_angles.pitch - cam_pose.body_offset.pitch)
                     .rotated_z(-cam_pose.head_angles.yaw)
                     .rotated_y(-cam_pose.head_angles.pitch - (cam_pose.cam_id == CamID::UPPER
                                                                       ? upper_cam_pitch + cam_pose.head_offset.pitch
                                                                       : lower_cam_pitch))
                     .rotated_x(-(cam_pose.cam_id == CamID::UPPER ? cam_pose.head_offset.roll : 0));
        p2 = p2.rotated_x(-cam_pose.body_angles.roll - cam_pose.body_offset.roll)
                     .rotated_y(-cam_pose.body_angles.pitch - cam_pose.body_offset.pitch)
                     .rotated_z(-cam_pose.head_angles.yaw)
                     .rotated_y(-cam_pose.head_angles.pitch - (cam_pose.cam_id == CamID::UPPER
                                                                       ? upper_cam_pitch + cam_pose.head_offset.pitch
                                                                       : lower_cam_pitch))
                     .rotated_x(-(cam_pose.cam_id == CamID::UPPER ? cam_pose.head_offset.roll : 0));
    }
    if (p1.x <= 0 || p2.x <= 0)
        return {};
    float z1 = cam_depth / p1.x;
    float z2 = cam_depth / p2.x;
    Line p(-p1.y * z1 + cam_width / 2, -p1.z * z1 + cam_height / 2, -p2.y * z2 + cam_width / 2,
           -p2.z * z2 + cam_height / 2);
    if (auto l1 = Line({0, 0}, {0, cam_height}).intersect(p))
        if (auto l2 = Line({cam_width, 0}, {cam_width, cam_height}).intersect(p))
            return Line(*l1, *l2);
    return {};
}

bool LocalizationUtils::belowHorizon(const point_2d& p, const Line& horizon) {
    return (p.x - horizon.px1) * (horizon.py1 - horizon.py2) + (p.y - horizon.py1) * (horizon.px2 - horizon.px1) > 0;
}

optional<float> LocalizationUtils::getObjectDist(const point_2d& p_, float height_above_ground,
                                                 const CamPose& cam_pose) {
    if (cam_pose.v5_angles)
        return LocalizationUtils::getObjectDistV5(p_, height_above_ground, cam_pose);
    testTranslation(cam_pose.translation);

    point_3d p(cam_depth, -p_.x + cam_width / 2, -p_.y + cam_height / 2);
    if (cam_pose.ellipse_angles)
        p = p.rotated_y(cam_pose.ellipse_angles->pitch)
                    .rotated_x(cam_pose.ellipse_angles->roll)
                    .rotated_z(cam_pose.head_angles.yaw);
    else
        p = p.rotated_x(cam_pose.cam_id == CamID::UPPER ? cam_pose.head_offset.roll : 0)
                    .rotated_y(cam_pose.head_angles.pitch + (cam_pose.cam_id == CamID::UPPER
                                                                     ? upper_cam_pitch + cam_pose.head_offset.pitch
                                                                     : lower_cam_pitch))
                    .rotated_z(cam_pose.head_angles.yaw)
                    .rotated_y(cam_pose.body_angles.pitch + cam_pose.body_offset.pitch)
                    .rotated_x(cam_pose.body_angles.roll + cam_pose.body_offset.roll);
    if (p.z >= 0)
        return {};
    // We're projecting an object above the ground, so we "move" the camera down.
    float n = -(cam_pose.translation.z - height_above_ground) / p.z;
    return p.norm() * n;
}

optional<float> LocalizationUtils::getPixelRadius(const point_2d& p, const CamPose& cam_pose, float obj_radius) {
    if (auto dist_cam = getObjectDist(p, obj_radius, cam_pose)) {
        if (*dist_cam == 0.f)
            return {};

        return getPixelRadius(*dist_cam, cam_pose, obj_radius);
    }
    return {};
}

float LocalizationUtils::getPixelRadius(float distance, const CamPose& cam_pose, float obj_radius) {
    return obj_radius / distance * (cam_pose.v5_angles ? cam_depth_v5 : cam_depth);
}

optional<point_2d> LocalizationUtils::projectV5(const point_2d& p_, const CamPose& cam_pose) {
    point_3d p(cam_depth_v5, -p_.x + cam_width / 2, -p_.y + cam_height / 2);
    p = p.rotated_x(cam_pose.v5_angles->roll).rotated_y(cam_pose.v5_angles->pitch);
    if (p.z >= 0)
        return {};
    p *= -.5f / p.z;
    return point_2d{p.x, p.y};
}

optional<Line> LocalizationUtils::getHorizonV5(const CamPose& cam_pose) {
    // This is by far not the most efficient way to do this but we've got the CPU power. :)
    point_3d p1 = point_3d(1, 0, 0).rotated_z(1_deg);
    point_3d p2 = point_3d(1, 0, 0).rotated_z(-1_deg);
    p1 = p1.rotated_x(-cam_pose.v5_angles->roll).rotated_y(-cam_pose.v5_angles->pitch);
    p2 = p2.rotated_x(-cam_pose.v5_angles->roll).rotated_y(-cam_pose.v5_angles->pitch);
    if (p1.x <= 0 || p2.x <= 0)
        return {};
    float z1 = cam_depth_v5 / p1.x;
    float z2 = cam_depth_v5 / p2.x;
    Line p(-p1.y * z1 + cam_width / 2, -p1.z * z1 + cam_height / 2, -p2.y * z2 + cam_width / 2,
           -p2.z * z2 + cam_height / 2);
    if (auto l1 = Line({0, 0}, {0, cam_height}).intersect(p))
        if (auto l2 = Line({cam_width, 0}, {cam_width, cam_height}).intersect(p))
            return Line(*l1, *l2);
    return {};
}

optional<float> LocalizationUtils::getObjectDistV5(const point_2d& p_, float height_above_ground,
                                                   const CamPose& cam_pose) {
    point_3d p(cam_depth_v5, -p_.x + cam_width / 2, -p_.y + cam_height / 2);
    p = p.rotated_x(cam_pose.v5_angles->roll).rotated_y(cam_pose.v5_angles->pitch);
    if (p.z >= 0)
        return {};
    // We're projecting an object above the ground, so we "move" the camera down.
    float n = -(.5f - height_above_ground) / p.z;
    return p.norm() * n;
}
