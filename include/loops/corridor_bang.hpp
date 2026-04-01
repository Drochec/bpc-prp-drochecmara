#pragma once

#include <helper.hpp>
#include <string>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "kinematics.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

using namespace std::chrono_literals;
namespace loops {

    constexpr float forward_speed_corridor = 0.075;


    class CorridorBang : public rclcpp::Node {

        algorithms::RobotSpeed cmd_vel_;

        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriber_range_est_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_cmd_vel_;
        rclcpp::TimerBase::SharedPtr timer_;
    public:
        CorridorBang() : rclcpp::Node("bang_bang"), cmd_vel_({forward_speed_corridor,0}) {

            subscriber_range_est_ = create_subscription<std_msgs::msg::Float32MultiArray>(
                Topic::range_estimate,
                15,
                std::bind(&CorridorBang::corridor_est_discrete_callback,this, std::placeholders::_1)
            );

            publisher_cmd_vel_ = create_publisher<std_msgs::msg::Float32MultiArray>(Topic::cmd_vel,5);

            timer_ = create_wall_timer(25ms, std::bind(&CorridorBang::publish_cmd_vel,this));

        }

    public:

        void publish_cmd_vel();

        void corridor_est_discrete_callback(std_msgs::msg::Float32MultiArray::SharedPtr msg);

    };
}