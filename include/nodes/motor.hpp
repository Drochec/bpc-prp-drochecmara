#pragma once

#include <string>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <helper.hpp>

#include "kinematics.hpp"

using namespace std::chrono_literals;

namespace nodes {

    float constexpr max_speed = 19.5f; //rad/s
    class MotorNode : public rclcpp::Node {
    private:
        // Subscriber
        rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr subscriber_;
        // Subscriber
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriber_cmd_vel_;
        // Publisher
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        algorithms::WheelSpeed wheel_speed_;
        algorithms::Encoders encoders_;
    public:
        // Constructor
        MotorNode() : rclcpp::Node("Motor_node"), wheel_speed_({0,0}), encoders_({0,0})
        {
            publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(Topic::set_motor_speeds, 10);

            subscriber_ = this->create_subscription<std_msgs::msg::UInt32MultiArray>(
                            Topic::encoders,
                            1,
                            std::bind(&MotorNode::encoder_callback, this, std::placeholders::_1)
                        );

            subscriber_cmd_vel_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
                Topic::cmd_vel,
                1,
                std::bind(&MotorNode::cmd_vel_callback, this, std::placeholders::_1));

            timer_ = this->create_wall_timer(100ms,std::bind(&MotorNode::set_speed_callback, this));
        }

    private:
        void set_speed_callback();

        void encoder_callback(const std_msgs::msg::UInt32MultiArray::SharedPtr msg);

        void cmd_vel_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    };
}