#pragma once

#include <string>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <helper.hpp>

#include "algorithms/kinematics.hpp"

using namespace std::chrono_literals;

namespace nodes {

    class MotorNode : public rclcpp::Node {
    private:
        // Subscriber
        rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr subscriber_;
        // Subscriber
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriber_cmd_vel_;
        // Publisher
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr coords_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr coords_pub_timer_;

        algorithms::Kinematics kinematics_; //Treba instace jelikoz Kinematics funkce nejsou static - mozna predelat?
        algorithms::RobotSpeed cmd_vel_; //Nastavena rychlost
        algorithms::Encoders encoders_; //Prectene hodnoty enkoderu
        algorithms::Coordinates act_coords_;

        float distance_driven_;

    public:
        // Constructor
        MotorNode();

    private:
        void set_speed_callback();

        void publish_coords_cb();

        void encoder_callback(const std_msgs::msg::UInt32MultiArray::SharedPtr msg);

        void cmd_vel_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    };
}