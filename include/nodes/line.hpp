#pragma once


#include <iostream>
#include <string>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <helper.hpp>

// Public API sketch; adapt to your project
enum class DiscreteLinePose {
    LineOnLeft,
    LineOnRight,
    LineNone,
    LineBoth,
};

struct SensorVals {
    uint16_t left;
    uint16_t right;
};

struct SensorNorm {
    float left;
    float right;
};

using namespace std::chrono_literals;

namespace nodes {
    class LineNode : public rclcpp::Node {
        //TODO:
        //Calibration service

        rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr subscriber_;
        
        //Discrete Line Position Line publisher
        rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr publisher_line_discrete_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_line_;
        rclcpp::TimerBase::SharedPtr timer_;

        SensorVals sensor_vals_raw_;
        SensorVals sensor_max_ = {450, 600};
        SensorVals sensor_min_ = {25, 25};
        SensorNorm sensor_vals_;

    public:
        LineNode() : rclcpp::Node("line_node") {

            publisher_line_discrete_ = create_publisher<std_msgs::msg::UInt8>(Topic::line_estimate_discrete,10);
            publisher_line_ = create_publisher<std_msgs::msg::Float32>(Topic::line_estimate,10);

            subscriber_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
                    Topic::line,
                    1,
                    std::bind(&LineNode::on_line_sensors_msg, this, std::placeholders::_1)
                );

            timer_ = this->create_wall_timer(10ms,std::bind(&LineNode::publish_line_estimate, this));
            
        };

        // Relative pose to line [m]
        float get_continuous_line_pose() const;

        DiscreteLinePose get_discrete_line_pose() const;

        void publish_line_estimate() const;

    private:

        void on_line_sensors_msg(const std_msgs::msg::UInt16MultiArray::SharedPtr msg);

    };
}

namespace algorithms {
    float constexpr sensor_offset = 0.0145; //sirsi rozostup
    //float constexpr sensor_offset = 0.0075; //pri sebe
    float constexpr treshold = 0.0005;     //treba normovat

    class LineEstimator {
    public:
        static DiscreteLinePose estimate_discrete_line_pose(const SensorNorm& sensor_vals);

        static float estimate_continuous_line_pose(const SensorNorm& sensor_vals);
    };
}