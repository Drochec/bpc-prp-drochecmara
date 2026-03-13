#pragma once

#include <helper.hpp>
#include <string>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>

namespace loops {
    class BangBang : public rclcpp::Node {

        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscriber_;
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        BangBang() : Node("bang_bang") {

        }
    };
}