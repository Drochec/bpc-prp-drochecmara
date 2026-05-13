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
//#include <std_msgs/msg/detail/float32__struct.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include "pid.hpp"
#include "prp_project/srv/calibrate_trigger.hpp"
#include "prp_project/srv/reset_yaw_trigger.hpp"
#include "prp_project/srv/button_cmd.hpp"
//#include "line.hpp"

using namespace std::chrono_literals;

namespace loops {
    enum class corridor_state {
        WAIT,
        CALIBRATION,
        CORRIDOR_FOLLOWING,
        CENTERING,
        INTERSECTION,
        INTERSECTION_ADVANCE,
        EXIT_INTERSECTION,
        TURNING,
        RESET,
    };

    class CorridorNav : public rclcpp::Node {

        algorithms::RobotSpeed cmd_vel_;
        algorithms::LidarFilterResults lidar_vals_;
        algorithms::LidarFilterResults intersection_vals_;
        float yaw_estimate_;
        float set_yaw_;
        bool exiting_corridor_;
        uint8_t line_detection_;
        algorithms::Coordinates act_coords_;
        
        algorithms::Encoders encoders_;
        algorithms::Encoders last_encoders_;
        float distance_traveled_at_intersection_;
        corridor_state next_turn_direction_state_;

        corridor_state state_;
        corridor_state last_state_;
        unsigned char reg_mode_;

        algorithms::Kinematics kinematics_;
        algorithms::Pid pid_yaw_;
        algorithms::Pid pid_yaw_turn_;
        algorithms::Pid pid_centering_;

        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriber_range_est_;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriber_intersection_range_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscriber_yaw_est_;
        rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr subscriber_encoders_;
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscriber_line_;
        //rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriber_coords_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_cmd_vel_;

        rclcpp::TimerBase::SharedPtr publish_timer_;
        rclcpp::TimerBase::SharedPtr decision_timer_;


        rclcpp::Service<prp_project::srv::ButtonCmd>::SharedPtr button_cmd_service_;

        rclcpp::Client<prp_project::srv::CalibrateTrigger>::SharedPtr calibrate_client_;
        rclcpp::Client<prp_project::srv::ResetYawTrigger>::SharedPtr reset_yaw_client_;


        void publish_cmd_vel();

        void range_est_callback(std_msgs::msg::Float32MultiArray::SharedPtr msg);

        void intersection_range_callback(std_msgs::msg::Float32MultiArray::SharedPtr msg);

        void yaw_est_callback(std_msgs::msg::Float32::SharedPtr msg);

        void coords_callback(std_msgs::msg::Float32MultiArray::SharedPtr msg);

        void encoders_callback(std_msgs::msg::UInt32MultiArray::SharedPtr msg);

        void line_callback(std_msgs::msg::UInt8::SharedPtr msg);

        void state_machine();

        void button_cmd_handle(
            const std::shared_ptr<prp_project::srv::ButtonCmd::Request> request,
            std::shared_ptr<prp_project::srv::ButtonCmd::Response> response
        );

        void send_calibrate_trigger();

        void send_reset_yaw();

        void centering_setup();

    public:

    CorridorNav();
    };
}