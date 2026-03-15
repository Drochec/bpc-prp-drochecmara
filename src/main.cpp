#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include "RosExampleClass.h"
#include "helper.hpp"
#include "nodes/line.hpp"
#include "nodes/io_node.hpp"
#include "nodes/motor.hpp"
#include "algorithms/kinematics.hpp"
#include "nodes/gamepad_node.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    // Create nodes
    
    auto io_node = std::make_shared<nodes::IoNode>();
    auto motor_node = std::make_shared<nodes::MotorNode>();
    auto line_node = std::make_shared<nodes::LineNode>();
    auto joy_node = std::make_shared<nodes::JoyNode>();
   
    
    //Setup ROS and spin nodes
    executor->add_node(io_node);
    executor->add_node(motor_node);
    executor->add_node(line_node);
    executor->add_node(joy_node);

    executor->spin();

    rclcpp::shutdown();
    return 0;
}
