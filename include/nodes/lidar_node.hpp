#pragma once

#include <cmath>
#include <vector>
#include <numeric>
#include <rclcpp/rclcpp.hpp>
#include "helper.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <chrono>

#include "kinematics.hpp"

namespace algorithms {

    // Structure to store filtered average distances in key directions
    struct LidarFilterResults {
        float front;
        float back;
        float left;
        float right;
    };

    class LidarFilter {
    public:
        LidarFilter() = default;

        static LidarFilterResults apply_filter(std::vector<float> points, float angle_start, float angle_end, float range_max, float range_min);
        static LidarFilterResults apply_filter_intersection(std::vector<float> points, float angle_start, float angle_end, float range_max, float range_min);
    };
}

namespace nodes {

    using namespace std::chrono_literals;
    class LidarNode : public rclcpp::Node {

        rclcpp::Time last_lidar_msg_time_;

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_intersect_;
        rclcpp::TimerBase::SharedPtr timer_;

        algorithms::LidarFilterResults lidar_filter_results_;
        algorithms::LidarFilterResults lidar_filter_intersect_results_;

    public:
        LidarNode();
        void publish();
        void subscriber_callback(sensor_msgs::msg::LaserScan::SharedPtr msg);

    };
}


