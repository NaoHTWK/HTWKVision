#pragma once

#include <point_2d.h>
#include <point_3d.h>

enum class CamID { UPPER = 0, LOWER = 1 };

constexpr int cam_width = 640;
constexpr int cam_height = 480;
constexpr htwk::point_2d cam_res{cam_width, cam_height};
constexpr float cam_fov = 0.952934839f;               // 0.98262f;
constexpr float cam_tan_fov_2 = 0.516129032f;  // tan(fov/2)
constexpr float cam_depth = cam_width / 2.f / cam_tan_fov_2;
// Length of the torso from hip to neck.
constexpr float torso_length = 0.2115f;
// The height of the kinematics in the walking engine ignores the foot, so we need to add it later.
constexpr float foot_height = 0.0452f;

constexpr float lower_cam_pitch = 0.67f;
#ifdef WEBOTS
constexpr float upper_cam_pitch = 0.030943951f;
#else
constexpr float upper_cam_pitch = -0.05f;  // 0.020943951f
#endif
// Relative position of lower cam to the neck joint
constexpr point_3d lower_cam_neck{.05071f, 0, .023f};
// Relative position of upper cam to the neck joint
constexpr point_3d upper_cam_neck{.05871f, 0, .071f};
