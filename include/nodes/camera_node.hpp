#pragma once

#include "aruco_detector.hpp"
#include <sensor_msgs/msg/compressed_image.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include "helper.hpp"

using namespace std::chrono_literals;
namespace nodes {

    class CameraNode : public rclcpp::Node {

        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr camera_subscriber_;
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        std::vector<algorithms::ArucoDetector::Aruco> aruco_;
        algorithms::ArucoDetector aruco_detector_;
        cv::Mat img_;

        void on_cam_msg(const sensor_msgs::msg::CompressedImage::SharedPtr msg);

        void publish_detections();

    public:

        CameraNode();
    };
}
