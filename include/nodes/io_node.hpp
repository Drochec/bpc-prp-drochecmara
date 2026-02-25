#pragma once

#include <iostream>
#include <string>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <helper.hpp>

using namespace std::chrono_literals;

namespace nodes {
    class IoNode : public rclcpp::Node {
    private:
        // Subscriber
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscriber_;
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

    public:
        // Constructor
        IoNode() : rclcpp::Node("io_node")
        {
            publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(Topic::set_rgb_leds, 10);

            subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(
                Topic::buttons,
                1,
                std::bind(&IoNode::on_button_callback, this, std::placeholders::_1)
            );


            timer_ = this->create_wall_timer(500ms,std::bind(&IoNode::rgb_timer_callback, this));
        }

        // Destructor (default)
        ~IoNode() override = default;

        // Function to retrieve the last pressed button value
        int get_button_pressed() const;



    private:
        // Variable to store the last received button press value
        int button_pressed_ = -1;

        // Subscriber for button press messages
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr button_subscriber_;

        // Callback - preprocess received message
        void on_button_callback(const std_msgs::msg::UInt8::SharedPtr msg);

        void rgb_timer_callback();
    };

}
