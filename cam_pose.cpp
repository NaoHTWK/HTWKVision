#include "cam_pose.h"

void CamPose::calcCamTranslation() {
    // Go straight up from the ground to the hip.
    translation = point_3d(0, 0, leg_height + foot_height);
    // Add the torso which is rotated by body_angles.
    translation += point_3d(0, 0, torso_length).rotated_y(body_angles.pitch).rotated_x(body_angles.roll);
    // Add the vector to the cam, rotated by body_angles and head_Cangles.
    translation += (cam_id == CamID::UPPER ? upper_cam_neck : lower_cam_neck)
                           .rotated_y(head_angles.pitch)
                           .rotated_z(head_angles.yaw)
                           .rotated_y(body_angles.pitch + body_offset.pitch)
                           .rotated_x(body_angles.roll + body_offset.roll);
}
