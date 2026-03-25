#pragma once

#include <helper.hpp>
#include <string>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "helper.hpp"
#include "kinematics.hpp"
#include "pid.hpp"
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

using namespace std::chrono_literals;
namespace loops {

    constexpr float forward_speed_pid = 0.075;

    constexpr float Kp = 15;
    constexpr float Ki = 0;
    constexpr float Kd = 0.5;


    class PidNode : public rclcpp::Node {

        algorithms::Pid pid_;
        algorithms::RobotSpeed cmd_vel_;
        double line_pose_;

        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscriber_line_est_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_cmd_vel_;
        rclcpp::TimerBase::SharedPtr timer_;
    public:
        PidNode() : rclcpp::Node("pid_control"), pid_(Kp,Ki,Kd), cmd_vel_({forward_speed_pid,0}), line_pose_(0) {
            
            subscriber_line_est_ = create_subscription<std_msgs::msg::Float32>(
                Topic::line_estimate,
                15,
                std::bind(&PidNode::line_est_callback,this, std::placeholders::_1)
            );

            publisher_cmd_vel_ = create_publisher<std_msgs::msg::Float32MultiArray>(Topic::cmd_vel,10);

            timer_ = create_wall_timer(5ms, std::bind(&PidNode::publish_cmd_vel,this));

        }

        void publish_cmd_vel();

        void line_est_callback(std_msgs::msg::Float32::SharedPtr msg);

    };
}