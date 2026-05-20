#pragma once

#include <iostream>
#include <string>

static const int MAIN_LOOP_PERIOD_MS = 50;

namespace Topic {
    //IO
    const std::string buttons = "/bpc_prp_robot/buttons";
    const std::string set_rgb_leds = "/bpc_prp_robot/rgb_leds";
    //const std::string machine_state = "/bpc_prp_robot/machine_state";
    //Motors
    const std::string set_motor_speeds = "/bpc_prp_robot/set_motor_speeds";
    const std::string cmd_vel = "/controller/cmd_vel";
    const std::string encoders = "/bpc_prp_robot/encoders";
    const std::string coords = "/controller/coords";
    //Line
    const std::string line = "/bpc_prp_robot/line_sensors";
    const std::string line_estimate_discrete = "/controller/line_estimate_discrete";
    const std::string line_estimate = "/controller/line_estimate";
    //Lidar
    const std::string lidar = "/bpc_prp_robot/lidar";
    const std::string range_estimate = "/controller/range_estimate";
    const std::string intersect_estimate = "/controller/intersect_estimate";
    //IMU
    const std::string imu = "/bpc_prp_robot/imu";
    const std::string yaw_estimate = "/controller/yaw_estimate";
    //Camera
    const std::string camera = "/bpc_prp_robot/camera/compressed";
    const std::string tag_detections = "/controller/tag_detections";

    //Corridor Navigation
    const std::string state = "/controller/state";

};

namespace Frame {
    const std::string origin = "origin";
    const std::string robot = "robot";
    const std::string lidar = "lidar";
};
