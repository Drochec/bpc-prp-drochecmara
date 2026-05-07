#include "camera_node.hpp"

namespace nodes {

    CameraNode::CameraNode() : rclcpp::Node("camera_node") {

    camera_subscriber_ = create_subscription<sensor_msgs::msg::CompressedImage>(
        Topic::camera,
        10,
        std::bind(&CameraNode::on_cam_msg, this, std::placeholders::_1));
    //publisher_ = create_publisher<std_msgs::msg::Float32>(Topic::yaw_estimate,10);
    //timer_ = create_wall_timer(20ms, std::bind(&CameraNode::publish_estimate, this));

    }

    void CameraNode::on_cam_msg(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        auto img = cv::imdecode(msg->data, cv::IMREAD_COLOR);

        if (!img.empty()) {
            img_ = img;
            aruco_ = aruco_detector_.detect(img);

            if (!aruco_.empty()) {
                RCLCPP_INFO(get_logger(),"ID: %d",aruco_[0].id);
            }
        }
    }

}