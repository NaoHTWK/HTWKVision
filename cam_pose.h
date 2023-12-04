#pragma once

#include <imu.h>
#include <cam_constants.h>
#include <cam_offset.h>

class CamPose {
public:
    CamPose(float leg_height, YPR body_angles, YawPitch head_angles, CamID cam_id,
            PitchRoll body_offset = PitchRoll(CamOffset::bodyPitch, CamOffset::bodyRoll),
            PitchRoll head_offset = PitchRoll(CamOffset::headPitch, CamOffset::headRoll))
        : cam_id(cam_id),
          head_angles(head_angles),
          leg_height(leg_height),
          body_angles(body_angles),
          body_offset(body_offset),
          head_offset(head_offset) {
        calcCamTranslation();
    }

    // Create a CamPose from the data we have in V5 images. Don't use on V6!
    // Roll in the old pf_angles was inverted compared to our current coordinate system.
    // This also ignores any head yaw, implement if needed.
    static CamPose fromV5(PitchRoll pf_angles) {
        CamPose cam_pose;
        cam_pose.v5_angles = PitchRoll(pf_angles.pitch, -pf_angles.roll);
        return cam_pose;
    }

    CamPose() = default;
    CamPose(const CamPose& other) = default;
    CamPose(CamPose&& other) = default;
    CamPose& operator=(const CamPose&) = default;
    CamPose& operator=(CamPose&&) = default;

    void setEllipseAngles(PitchRoll angles) {
        ellipse_angles = angles;
    }
    void removeEllipseAngles() {
        ellipse_angles = std::nullopt;
    }
    const point_3d& get_translation() const {
        return translation;
    }

    CamID cam_id = CamID::UPPER;
    YawPitch head_angles;

private:
    void calcCamTranslation();

    float leg_height = 0;
    YPR body_angles;
    PitchRoll body_offset;
    PitchRoll head_offset;
    std::optional<PitchRoll> ellipse_angles;
    point_3d translation;

    std::optional<PitchRoll> v5_angles;

    friend class LocalizationUtils;
    friend class VisionLogger;
};
