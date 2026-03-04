#pragma once

#include <string>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <helper.hpp>

using namespace std::chrono_literals;

namespace nodes {
    class Motor_Node : public rclcpp::Node {
    private:
        // Subscriber
        rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr subscriber_;
        // Publisher
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
    public:
        // Constructor
        Motor_Node() : rclcpp::Node("Motor_node")
        {
            publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(Topic::set_motor_speeds, 10);

            subscriber_ = this->create_subscription<std_msgs::msg::UInt32MultiArray>(
                            Topic::encoders,
                            1,
                            std::bind(&Motor_Node::encoder_callback, this, std::placeholders::_1)
                        );

            timer_ = this->create_wall_timer(100ms,std::bind(&Motor_Node::set_speed_callback, this));
        }

        // Destructor (default)
        ~Motor_Node() override = default;



    private:
        void set_speed_callback();

        void encoder_callback(const std_msgs::msg::UInt32MultiArray::SharedPtr msg);
    };
}