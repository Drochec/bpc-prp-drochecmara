#include "camera_node.hpp"

namespace nodes {

    CameraNode::CameraNode() : rclcpp::Node("camera_node") {

    camera_subscriber_ = create_subscription<sensor_msgs::msg::CompressedImage>(
        Topic::camera,
        10,
        std::bind(&CameraNode::on_cam_msg, this, std::placeholders::_1));
    publisher_ = create_publisher<std_msgs::msg::UInt8MultiArray>(Topic::tag_detections,10);
    timer_ = create_wall_timer(50ms, std::bind(&CameraNode::publish_detections, this));

    }

    void CameraNode::on_cam_msg(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        auto img = cv::imdecode(msg->data, cv::IMREAD_COLOR);

        if (!img.empty()) {
            img_ = img;
            aruco_ = aruco_detector_.detect(img);


            /*
            if (!aruco_.empty()) {
                RCLCPP_INFO(get_logger(),"ID: %d",aruco_[0].id);
            }*/
        }
    }

    void CameraNode::publish_detections() {

            auto msg = std_msgs::msg::UInt8MultiArray();

            msg.data.reserve(aruco_.size());

        for (const auto& aruco : aruco_) {
            msg.data.push_back(static_cast<uint8_t>(aruco.id));
        }

            publisher_->publish(msg);

        
    }

}