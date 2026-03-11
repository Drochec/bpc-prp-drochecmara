#include "nodes/motor.hpp"
namespace nodes {
    int i = 0;

    void MotorNode::set_speed_callback() {
        auto msg = std_msgs::msg::UInt8MultiArray();

        //Prepocet na hodnoty do driveru
        auto w_l = static_cast<u_int8_t>((255-127)/max_speed*wheel_speed_.l + 127);
        auto w_r = static_cast<u_int8_t>((255-127)/max_speed*wheel_speed_.r + 127);

        msg.data={w_l, w_r};

        publisher_->publish(msg);
        //RCLCPP_INFO(this->get_logger(), "Sending speeeed %d %d",i,i);
    }

    void MotorNode::encoder_callback(const std_msgs::msg::UInt32MultiArray::SharedPtr msg) {
        encoders_.l = msg->data[0];
        encoders_.r = msg->data[1];
        //RCLCPP_INFO(this->get_logger(), "Receiving encoder data %u %u",msg->data[0],msg->data[1]);
    }


}