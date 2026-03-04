#include "nodes/motor.hpp"
namespace nodes {
    int i = 0;

    void Motor_Node::set_speed_callback() {
        auto msg = std_msgs::msg::UInt8MultiArray();

        if (i >= 255){
            i = 255;
        }
        else {
            i++;
        }
        msg.data={i,i};

        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Sending speeeed %d %d",i,i);
    }

    void Motor_Node::encoder_callback(const std_msgs::msg::UInt32MultiArray::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Receiving encoder data %u %u",msg->data[0],msg->data[1]);
    }


}