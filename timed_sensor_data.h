#pragma once

#include "battery.h"

#include <imu.h>
#include <joints.h>
#include <position.h>
#include <PositionPlain.h>

struct TimedSensorData {
    YPR body_angles;
    HeadJointAngles head{0, 0};
    Position odo;
    float leg_height{};  // Height of the leg according to the walking engine.
    int64_t time = 0;  // in us
    bool isStanding{false};
    float walk_phase{};
    float step_height{};
    Battery battery;
    IMU imu;
    kinematics::PositionPlain feet;
};
