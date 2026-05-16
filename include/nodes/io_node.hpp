#pragma once

#include <iostream>
#include <string>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <helper.hpp>
#include "prp_project/srv/button_cmd.hpp"

using namespace std::chrono_literals;

namespace nodes {
    class IoNode : public rclcpp::Node {
    private:
        // Subscriber
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscriber_;
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;

        rclcpp::TimerBase::SharedPtr timer_;

        // Service
        rclcpp::Client<prp_project::srv::ButtonCmd>::SharedPtr button_cmd_client_;


        // Variable to store the last received button press value
        unsigned int button_pressed_;
        

        // Callback - preprocess received message
        void on_button_callback(const std_msgs::msg::UInt8::SharedPtr msg);

        void rgb_timer_callback();

        void send_button_cmd(unsigned int button);

    public:
        // Constructor
        IoNode();
        // Destructor (default)
        ~IoNode() override = default;

        // Function to retrieve the last pressed button value
        int get_button_pressed() const;

    };

}
