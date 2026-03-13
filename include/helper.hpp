#pragma once

#include <iostream>
#include <string>

static const int MAIN_LOOP_PERIOD_MS = 50;

namespace Topic {
    const std::string buttons = "/bpc_prp_robot/buttons";
    const std::string set_rgb_leds = "/bpc_prp_robot/rgb_leds";
    const std::string set_motor_speeds = "/bpc_prp_robot/set_motor_speeds";
    const std::string encoders = "/bpc_prp_robot/encoders";
    const std::string line = "/bpc_prp_robot/line_sensors";
    const std::string cmd_vel = "/bpc_prp_robot/cmd_vel";
    const std::string line_estimate_discrete = "/bpc_prp_robot/line_estimate_discrete";
    const std::string line_estimate = "/bpc_prp_robot/line_estimate";
};

namespace Frame {
    const std::string origin = "origin";
    const std::string robot = "robot";
    const std::string lidar = "lidar";
};
