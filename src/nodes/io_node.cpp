#include "nodes/io_node.hpp"
namespace nodes {


    int IoNode::get_button_pressed() const {
        return button_pressed_;
    }
    void IoNode::on_button_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Button pressed: %d", msg->data);
        button_pressed_ = msg->data;
        //this->rgb_timer_callback();
    }

    void IoNode::rgb_timer_callback() {
      auto msg = std_msgs::msg::UInt8MultiArray();

        auto btn_state = get_button_pressed();
        if (btn_state == 0) {
            msg.data = {255, 0, 0,
            255, 0, 0,
            255, 0, 0,
            255, 0, 0};
        }
        else if (btn_state == 1) {
            msg.data = {0, 255, 0,
            0, 255, 0,
            0, 255, 0,
            0, 255, 0};
        }
        else if (btn_state == 2) {
            msg.data = {0, 0, 255,
            0, 0, 255,
            0, 0, 255,
            0, 0, 255};
        }

        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Sending LED data");
    }




}
