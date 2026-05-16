#include "nodes/io_node.hpp"
#include "loops/corridor_nav.hpp"
namespace nodes {

    IoNode::IoNode() : rclcpp::Node("io_node"), button_pressed_(0)
        {
            publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(Topic::set_rgb_leds, 10);

            subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(
                Topic::buttons,
                1,
                std::bind(&IoNode::on_button_callback, this, std::placeholders::_1)
            );


            timer_ = this->create_wall_timer(500ms,std::bind(&IoNode::rgb_timer_callback, this));

            button_cmd_client_ = create_client<prp_project::srv::ButtonCmd>("button_cmd");
        }

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
            msg.data = {127, 0, 0,
            127, 0, 0,
            127, 0, 0,
            127, 0, 0};
        }
        else if (btn_state == 1) {
            msg.data = {0, 127, 0,
            0, 127, 0,
            0, 127, 0,
            0, 127, 0};
        }
        else if (btn_state == 0) {
            msg.data = {0, 0, 0,
            0, 0, 0,
            0, 0, 0,
            0, 0, 0};
        }

        publisher_->publish(msg);
        //RCLCPP_INFO(this->get_logger(), "Sending LED data");
    }




}
