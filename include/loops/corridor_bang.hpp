#pragma once

#include <helper.hpp>
#include <string>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "kinematics.hpp"
#include "lidar_node.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/detail/float32__struct.hpp>
#include "pid.hpp"

using namespace std::chrono_literals;
namespace loops {
    enum class corridor_state {
        CALIBRATION,
        CORRIDOR_FOLLOWING,
        CENTERING,
        INTERSECTION,
        EXIT_INTERSECTION,
        TURNING,
    };

    constexpr float forward_speed_corridor = 0.075;
    constexpr float wall_threshold = 0.6;
    constexpr float front_stop = 0.25;
    constexpr float exit_centering_error = 0.05;
    bool exiting_corridor = false;

    class CorridorBang : public rclcpp::Node {

        algorithms::RobotSpeed cmd_vel_;
        algorithms::LidarFilterResults lidar_vals_;
        float yaw_estimate_;
        float set_yaw_;

        corridor_state state_;
        algorithms::Pid pid_yaw_;
        algorithms::Pid pid_centering_;

        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriber_range_est_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscriber_yaw_est_;
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscriber_state_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_cmd_vel_;
        rclcpp::TimerBase::SharedPtr publish_timer_;
        rclcpp::TimerBase::SharedPtr decision_timer_;

    public:
        CorridorBang() : rclcpp::Node("bang_bang"),
                        cmd_vel_({0,0}),
                        lidar_vals_({0,0,0,0}),
                        yaw_estimate_(0),
                        set_yaw_(0),
                        state_(corridor_state::CALIBRATION),
                        pid_yaw_(3,0.3,0),
                        pid_centering_(10,0,1)
        {

            subscriber_range_est_ = create_subscription<std_msgs::msg::Float32MultiArray>(
                Topic::range_estimate,
                15,
                std::bind(&CorridorBang::corridor_est_discrete_callback,this, std::placeholders::_1)
            );

            subscriber_yaw_est_ = create_subscription<std_msgs::msg::Float32>(
                Topic::yaw_estimate,
                15,
                std::bind(&CorridorBang::yaw_est_callback,this, std::placeholders::_1)
            );

            subscriber_state_ = create_subscription<std_msgs::msg::UInt8>(
                Topic::machine_state,
                15,
                std::bind(&CorridorBang::set_state_callback,this, std::placeholders::_1)
            );

            publisher_cmd_vel_ = create_publisher<std_msgs::msg::Float32MultiArray>(Topic::cmd_vel,5);

            publish_timer_ = create_wall_timer(25ms, std::bind(&CorridorBang::publish_cmd_vel,this));

            decision_timer_ = create_wall_timer(30ms, std::bind(&CorridorBang::state_machine_driving,this));

        }

    public:

        void publish_cmd_vel();

        void corridor_est_discrete_callback(std_msgs::msg::Float32MultiArray::SharedPtr msg);

        void yaw_est_callback(std_msgs::msg::Float32::SharedPtr msg);

        void set_state_callback(std_msgs::msg::UInt8::SharedPtr msg);

        void state_machine_driving();

    };
}