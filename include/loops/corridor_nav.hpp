#pragma once

#include <helper.hpp>
#include <string>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include "kinematics.hpp"
#include "lidar_node.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/detail/float32__struct.hpp>
#include "pid.hpp"
#include "prp_project_nav_dev/srv/calibrate_trigger.hpp"
#include "prp_project_nav_dev/srv/reset_yaw_trigger.hpp"
#include "prp_project_nav_dev/srv/button_cmd.hpp"

using namespace std::chrono_literals;

namespace loops {
    enum class corridor_state {
        WAIT,
        CALIBRATION,
        CORRIDOR_FOLLOWING,
        CENTERING,
        INTERSECTION_APPROACH,
        INTERSECTION,
        INTERSECTION_ADVANCE,
        EXIT_INTERSECTION,
        TURNING,
        RESET,
    };

    constexpr float forward_speed_corridor = 0.075;
    constexpr float wall_threshold = 0.6;
    constexpr float front_stop = 0.25;
    constexpr float turn_check_angular_speed = 0.35;
    constexpr float max_corridor_angular_speed = 1.15;
    constexpr float max_turn_angular_speed = 1.4;
    constexpr float exit_centering_error = 0.05;
    constexpr float intersection_advance_distance = 0.12;  // 12cm in meters
    constexpr float encoder_wheel_radius = 68.55e-3f;
    constexpr int encoder_ticks_per_revolution = 585;

    class CorridorNav : public rclcpp::Node {

        algorithms::RobotSpeed cmd_vel_;
        algorithms::LidarFilterResults lidar_vals_;
        float yaw_estimate_;
        float set_yaw_;
        bool exiting_corridor_;

        corridor_state state_;
        corridor_state last_state_;
        corridor_state next_turn_direction_state_;

        algorithms::Pid pid_yaw_;
        algorithms::Pid pid_centering_;

        // Encoder data received from subscribers
        algorithms::Encoders current_encoders_;
        algorithms::Encoders encoders_at_intersection_start_;
        int intersection_turn_direction_;
        bool encoders_ready_;
        algorithms::Encoders intersection_start_encoders_;

        // Distance tracking
        float encoder_distance_total_ = 0.0f;
        float encoder_distance_at_intersection_start_ = 0.0f;
        float distance_traveled_at_intersection_ = 0.0f;

        // Subscribers
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriber_range_est_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscriber_yaw_est_;
        rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr subscriber_encoders_;
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscriber_state_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscriber_encoder_distance_;

        // Publishers
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_cmd_vel_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_distance_traveled_;

        rclcpp::TimerBase::SharedPtr publish_timer_;
        rclcpp::TimerBase::SharedPtr decision_timer_;

        rclcpp::Service<prp_project_nav_dev::srv::ButtonCmd>::SharedPtr button_cmd_service_;
        rclcpp::Client<prp_project_nav_dev::srv::CalibrateTrigger>::SharedPtr calibrate_client_;
        rclcpp::Client<prp_project_nav_dev::srv::ResetYawTrigger>::SharedPtr reset_yaw_client_;

        void publish_cmd_vel();

        void range_est_callback(std_msgs::msg::Float32MultiArray::SharedPtr msg);

        void yaw_est_callback(std_msgs::msg::Float32::SharedPtr msg);

        void encoders_callback(std_msgs::msg::UInt32MultiArray::SharedPtr msg);

        void set_state_callback(std_msgs::msg::UInt8::SharedPtr msg);

        void encoder_distance_callback(std_msgs::msg::Float32::SharedPtr msg);

        void state_machine();

        bool is_intersection(bool left_open, bool right_open, bool front_open) const;

        bool should_check_turn_context(float corridor_turn_rate, bool left_open, bool right_open, bool front_open) const;

        int choose_turn_direction(bool left_open, bool right_open, bool front_open) const;

        void prepare_intersection_decision(int turn_direction, bool advance_to_center);

        void button_cmd_handle(
            const std::shared_ptr<prp_project_nav_dev::srv::ButtonCmd::Request> request,
            std::shared_ptr<prp_project_nav_dev::srv::ButtonCmd::Response> response
        );

        void send_calibrate_trigger();

        void send_reset_yaw();

        //float normalize_angle(std_msgs::msg::Float32 set, std_msgs::msg::Float32 estimate);

        float normalize_angle(float angle)
        {
            return std::atan2(std::sin(angle), std::cos(angle));
        }

    public:

        CorridorNav() : rclcpp::Node("corridor_nav"),
                        cmd_vel_({0,0}),
                        lidar_vals_({0,0,0,0}),
                        yaw_estimate_(0),
                        set_yaw_(0),
                        exiting_corridor_(false),
                        state_(corridor_state::WAIT),
                        last_state_(corridor_state::RESET),
                        next_turn_direction_state_(corridor_state::TURNING),
                        pid_yaw_(3,0.3,0),
                        pid_centering_(9,0,0.04,0.08),
                        current_encoders_({0,0}),
                        encoders_at_intersection_start_({0,0}),
                        intersection_turn_direction_(0),
                        encoders_ready_(false)
        {

            subscriber_range_est_ = create_subscription<std_msgs::msg::Float32MultiArray>(
                Topic::range_estimate,
                15,
                std::bind(&CorridorNav::range_est_callback,this, std::placeholders::_1)
            );

            subscriber_yaw_est_ = create_subscription<std_msgs::msg::Float32>(
                Topic::yaw_estimate,
                15,
                std::bind(&CorridorNav::yaw_est_callback,this, std::placeholders::_1)
            );

            subscriber_state_ = create_subscription<std_msgs::msg::UInt8>(
                Topic::machine_state,
                15,
                std::bind(&CorridorNav::set_state_callback,this, std::placeholders::_1)
            );

            subscriber_encoders_ = create_subscription<std_msgs::msg::UInt32MultiArray>(
                Topic::encoders,
                15,
                std::bind(&CorridorNav::encoders_callback,this, std::placeholders::_1)
            );

            subscriber_encoder_distance_ = create_subscription<std_msgs::msg::Float32>(
                Topic::encoder_distance,
                15,
                std::bind(&CorridorNav::encoder_distance_callback,this,std::placeholders::_1)
            );

            publisher_cmd_vel_ = create_publisher<std_msgs::msg::Float32MultiArray>(Topic::cmd_vel,5);
            
            publisher_distance_traveled_ = create_publisher<std_msgs::msg::Float32>("/bpc_prp_robot/distance_traveled_intersection", 5);

            publish_timer_ = create_wall_timer(25ms, std::bind(&CorridorNav::publish_cmd_vel,this));

            decision_timer_ = create_wall_timer(30ms, std::bind(&CorridorNav::state_machine,this));

            calibrate_client_ = create_client<prp_project_nav_dev::srv::CalibrateTrigger>("calibrate");
            reset_yaw_client_ = create_client<prp_project_nav_dev::srv::ResetYawTrigger>("reset_yaw");

            button_cmd_service_ = create_service<prp_project_nav_dev::srv::ButtonCmd>(
                "button_cmd",
                std::bind(&CorridorNav::button_cmd_handle,this,std::placeholders::_1,std::placeholders::_2)
            );

        }

    };
}
