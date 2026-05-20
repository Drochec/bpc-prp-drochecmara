#include "nodes/io_node.hpp"
#include "loops/corridor_nav.hpp"

namespace nodes {

    IoNode::IoNode() : rclcpp::Node("io_node"), button_pressed_(0), 
    state_(IoNodeState::STATE),
    received_state_(loops::corridor_state::WAIT),
    received_paths_({false,false,false}),
    LED_({0,0,0,0,0,0,0,0,0,0,0,0})
        {
            publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(Topic::set_rgb_leds, 10);

            subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(
                Topic::buttons,
                1,
                std::bind(&IoNode::on_button_callback, this, std::placeholders::_1)
            );

            state_subscriber_ = create_subscription<std_msgs::msg::UInt8>(Topic::state, 1, std::bind(&IoNode::state_cb, this, std::placeholders::_1));

            path_subscriber_ = create_subscription<std_msgs::msg::UInt8MultiArray>(Topic::path, 1, std::bind(&IoNode::path_cb, this, std::placeholders::_1));

            timer_ = this->create_wall_timer(100ms,std::bind(&IoNode::rgb_timer_callback, this));

            path_display_timer_ = this->create_wall_timer(500ms,std::bind(&IoNode::end_display_path, this));

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

    void IoNode::state_cb(const std_msgs::msg::UInt8::SharedPtr msg) {
        received_state_ = static_cast<loops::corridor_state>(msg->data);
    }

    void IoNode::path_cb(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {

        for (size_t i = 0; i < msg->data.size(); i++) {
            received_paths_[i] = static_cast<bool>(msg->data[i]);
        }

        state_ = IoNodeState::PATH;
        path_display_timer_->reset();

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
        msg.data.reserve(12);

        if (state_ == IoNodeState::PATH){
            LED_[0] = colors::OFF;
            LED_[2] = received_paths_[2] ? colors::WHITE : colors::OFF;
            LED_[3] = received_paths_[0] ? colors::WHITE : colors::OFF;
            LED_[2] = received_paths_[1] ? colors::WHITE : colors::OFF;
        }
        else {  
            LED_[0] = state_to_color();
            LED_[1] = colors::OFF;
            LED_[2] = colors::OFF;
            LED_[3] = colors::OFF;

        }
        
        for (const auto& led : LED_) {
            msg.data.insert(msg.data.end(), led.begin(), led.end());
        }
        

        publisher_->publish(msg);
        //RCLCPP_INFO(this->get_logger(), "Sending LED data");
    }


    RGB IoNode::state_to_color() {
    switch (received_state_) {
        case loops::corridor_state::WAIT:                  return colors::WHITE;
        case loops::corridor_state::CALIBRATION:           return colors::BLUE;
        case loops::corridor_state::CORRIDOR_FOLLOWING:    return colors::YELLOW;
        case loops::corridor_state::INTERSECTION:          return colors::GREEN;
        case loops::corridor_state::INTERSECTION_ADVANCE:  return colors::RED;
        case loops::corridor_state::EXIT_INTERSECTION:     return colors::CYAN;
        case loops::corridor_state::TURNING:               return colors::MAGENTA;
        case loops::corridor_state::RESET:                 return colors::ORANGE;
    }

    return colors::OFF;
    }

    void IoNode::end_display_path(){
        state_ = IoNodeState::STATE;
        path_display_timer_->cancel();
    }





}
