#pragma once

#include <cam_constants.h>
#include <cam_offset.h>
#include <cam_pose.h>
#include <imu.h>
#include <line.h>
#include <point_2d.h>
#include <point_3d.h>
#include <position.h>

#include <optional>
#include <utility>

class LocalizationUtils {
public:
    static std::optional<htwk::point_2d> project(const htwk::point_2d& p_, float height, const CamPose& cam_pose);
    static std::optional<htwk::point_2d> project(const htwk::point_2d& p_, const CamPose& cam_pose);
    static std::optional<htwk::Line> project(const htwk::Line& l, const CamPose& cam_pose);
    // Transforms a relative point into a cam-centered coordinate system with x towards the depth and z up (i.e. -y in
    // image coordinates).
    static point_3d relToCam(point_3d p, const CamPose& cam_pose);
    static point_3d relToCam(const htwk::point_2d& p, const CamPose& cam_pose);
    static std::optional<htwk::point_2d> camToImage(point_3d p);
    static htwk::point_2d relToAbs(const htwk::point_2d& p, const Position& pos);
    static htwk::point_2d absToRel(const htwk::point_2d& p, const Position& pos);
    // Project points in a coordinate system with (0,0) in the center of the neck joint into the image.
    static std::optional<htwk::point_2d> neckToCam(point_3d p, const YawPitch& head_pos, CamID cam_id);
    // TODO: Test all the functions below on a robot.
    static std::optional<htwk::Line> getHorizon(const CamPose& cam_pose);
    static bool belowHorizon(const htwk::point_2d& p, const htwk::Line& horizon);
    static std::optional<float> getObjectDist(const htwk::point_2d& p, float height_above_ground,
                                              const CamPose& cam_pose);
    static std::optional<float> getPixelRadius(const htwk::point_2d& p, const CamPose& cam_pose, float obj_radius);
    static float getPixelRadius(float distance, const CamPose& cam_pose, float obj_radius);

private:
    static std::optional<htwk::point_2d> projectV5(const htwk::point_2d& p_, const CamPose& cam_pose);
    static std::optional<htwk::Line> getHorizonV5(const CamPose& cam_pose);
    static std::optional<float> getObjectDistV5(const htwk::point_2d& p, float height_above_ground,
                                                const CamPose& cam_pose);

    static constexpr float cam_depth_v5 = 565.42937385f;
};
