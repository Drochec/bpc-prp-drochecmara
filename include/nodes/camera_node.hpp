#pragma once

#include "aruco_detector.hpp"
#include <sensor_msgs/msg/compressed_image.hpp>
#include <rclcpp/rclcpp.hpp>

#include "helper.hpp"
using namespace std::chrono_literals;
namespace nodes {
    class CameraNode : public rclcpp::Node {
    private:
        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr camera_subscriber_;
        //rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
        //rclcpp::TimerBase::SharedPtr timer_;

        void on_cam_msg(const sensor_msgs::msg::CompressedImage::SharedPtr msg);

        std::vector<algorithms::ArucoDetector::Aruco> aruco_;
        algorithms::ArucoDetector aruco_detector_;
        cv::Mat img_;

    public:
        CameraNode() : rclcpp::Node("camera_node") {
            camera_subscriber_ = create_subscription<sensor_msgs::msg::CompressedImage>(
                Topic::camera,
                10,
                std::bind(&CameraNode::on_cam_msg, this, std::placeholders::_1));

            //publisher_ = create_publisher<std_msgs::msg::Float32>(Topic::yaw_estimate,10);
            //timer_ = create_wall_timer(20ms, std::bind(&CameraNode::publish_estimate, this));
        }
    };
}
