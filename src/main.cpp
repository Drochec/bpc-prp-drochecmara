#include <rclcpp/rclcpp.hpp>
#include <cmath>

#include "camera_node.hpp"
#include "corridor_nav.hpp"
#include "RosExampleClass.h"
#include "helper.hpp"
#include "imu_node.hpp"
#include "lidar_node.hpp"
#include "nodes/line.hpp"
#include "nodes/io_node.hpp"
#include "nodes/motor.hpp"
#include "algorithms/kinematics.hpp"
#include "nodes/gamepad_node.hpp"
#include "loops/line_loop_bang.hpp"
#include "loops/line_loop_pid.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    // Create nodes
    
    auto io_node = std::make_shared<nodes::IoNode>();
    auto motor_node = std::make_shared<nodes::MotorNode>();
    auto line_node = std::make_shared<nodes::LineNode>();
    auto imu_node = std::make_shared<nodes::ImuNode>();
    auto lidar_node = std::make_shared<nodes::LidarNode>();
    auto camera_node = std::make_shared<nodes::CameraNode>();
    //auto joy_node = std::make_shared<nodes::JoyNode>();
    //auto bangbang_node = std::make_shared<loops::BangBang>();
    auto corridor_nav_node = std::make_shared<loops::CorridorNav>();
    //auto pid_node = std::make_shared<loops::PidNode>();
    
    //Setup ROS and spin nodes
    executor->add_node(io_node);
    executor->add_node(motor_node);
    executor->add_node(line_node);
    executor->add_node(imu_node);
    executor->add_node(lidar_node);
    executor->add_node(camera_node);
    //executor->add_node(joy_node);
    //executor->add_node(bangbang_node);
    executor->add_node(corridor_nav_node);
    //executor->add_node(pid_node);

    executor->spin();

    rclcpp::shutdown();
    return 0;
}
