#pragma once

#include <helper.hpp>
#include <string>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "kinematics.hpp"
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

using namespace std::chrono_literals;
namespace loops {

    constexpr float forward_speed = 0.075;
    constexpr float turn_speed = 0.6;

    class BangBang : public rclcpp::Node {

        algorithms::RobotSpeed cmd_vel_;

        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscriber_line_est_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_cmd_vel_;
        rclcpp::TimerBase::SharedPtr timer_;
    public:
        BangBang() : rclcpp::Node("bang_bang"), cmd_vel_({forward_speed,0}) {
            
            subscriber_line_est_ = create_subscription<std_msgs::msg::UInt8>(
                Topic::line_estimate_discrete,
                15,
                std::bind(&BangBang::line_est_discrete_callback,this, std::placeholders::_1)
            );

            publisher_cmd_vel_ = create_publisher<std_msgs::msg::Float32MultiArray>(Topic::cmd_vel,5);

            timer_ = create_wall_timer(5ms, std::bind(&BangBang::publish_cmd_vel,this));

        }

        public:

        void publish_cmd_vel();

        void line_est_discrete_callback(std_msgs::msg::UInt8::SharedPtr msg);

    };
}