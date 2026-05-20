#pragma once

#include <iostream>
#include <string>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <helper.hpp>
#include "prp_project/srv/button_cmd.hpp"
#include "corridor_nav.hpp"

using namespace std::chrono_literals;

using RGB = std::array<uint8_t, 3>;

namespace colors {
    constexpr RGB OFF    = {1, 0, 0};
    constexpr RGB WHITE  = {255, 255, 255};
    constexpr RGB RED    = {255, 0, 0};
    constexpr RGB GREEN  = {0, 255, 0};
    constexpr RGB BLUE   = {0, 0, 255};
    constexpr RGB YELLOW = {255, 255, 0};
    constexpr RGB CYAN   = {0, 255, 255};
    constexpr RGB MAGENTA= {255, 0, 255};
    constexpr RGB ORANGE = {255, 165, 0};
    constexpr RGB PURPLE = {128, 0, 128};
}

namespace nodes {
    enum class IoNodeState {
        STATE,
        PATH
    };

    class IoNode : public rclcpp::Node {
    private:
        // Subscriber
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscriber_;
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr state_subscriber_;
        rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr path_subscriber_;
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr path_display_timer_;

        // Service
        rclcpp::Client<prp_project::srv::ButtonCmd>::SharedPtr button_cmd_client_;


        // Variable to store the last received button press value
        unsigned int button_pressed_;
        IoNodeState state_;
        loops::corridor_state received_state_; 
        std::array<bool, 3> received_paths_;
        std::array<RGB, 4> LED_;

        // Callback - preprocess received message
        void on_button_callback(const std_msgs::msg::UInt8::SharedPtr msg);

        void rgb_timer_callback();

        void send_button_cmd(unsigned int button);

        void state_cb(const std_msgs::msg::UInt8::SharedPtr msg);

        void path_cb(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);

        void end_display_path();

        std::array<uint8_t, 3> state_to_color();

    public:
        // Constructor
        IoNode();
        // Destructor (default)
        ~IoNode() override = default;

        // Function to retrieve the last pressed button value
        int get_button_pressed() const;

    };

}
