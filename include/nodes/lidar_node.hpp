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
    };
}

namespace nodes {

    using namespace std::chrono_literals;
    class LidarNode : public rclcpp::Node {

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        algorithms::LidarFilterResults lidar_filter_results_;


    public:
        LidarNode() : Node("lidar_node"), lidar_filter_results_({0, 0, 0, 0}) {

            publisher_ = create_publisher<std_msgs::msg::Float32MultiArray>(Topic::range_estimate,10);

            subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
                Topic::lidar,
                10,
                std::bind(&LidarNode::subscriber_callback, this, std::placeholders::_1));

            timer_ = create_wall_timer(5ms,std::bind(&LidarNode::publish,this));
        }

        void publish();
        void subscriber_callback(sensor_msgs::msg::LaserScan::SharedPtr msg);

    };
}


