#include "nodes/io_node.hpp"
#include "loops/corridor_nav.hpp"
namespace nodes {


    int IoNode::get_button_pressed() const {
        return button_pressed_;
    }
    void IoNode::on_button_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
        
        auto received_button = msg->data;

        //Debounce
        if (received_button != button_pressed_) {
            send_button_cmd(received_button);
            button_pressed_ = received_button;
        }

    }

    void IoNode::send_button_cmd(unsigned int button) {
        auto request = std::make_shared<prp_project::srv::ButtonCmd::Request>();

        if (button == 1) {
            request->command = "START";
            RCLCPP_INFO(get_logger(),"Sending START command");
        }
        else if (button == 2) {
            request->command = "STOP";
            RCLCPP_INFO(get_logger(),"Sending STOP command");
        }
        else {
            RCLCPP_WARN(get_logger(),"Wrong button id passed");
            return;
        }

        button_cmd_client_->async_send_request(request);
    }

    void IoNode::rgb_timer_callback() {
      auto msg = std_msgs::msg::UInt8MultiArray();
        auto machine_state = std_msgs::msg::UInt8();

        auto btn_state = get_button_pressed();
        if (btn_state == 2) {
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
        /*else if (btn_state == 0) {
            msg.data = {0, 0, 255,
            0, 0, 255,
            0, 0, 255,
            0, 0, 255};
        }*/

        publisher_->publish(msg);
        //RCLCPP_INFO(this->get_logger(), "Sending LED data");
    }




}
