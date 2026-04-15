#include "camera_node.hpp"

namespace nodes {
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