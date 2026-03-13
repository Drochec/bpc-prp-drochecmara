#include "nodes/motor.hpp"
#include <cstdint>

namespace nodes {
    int i = 0;

    void MotorNode::set_speed_callback() {
        auto msg = std_msgs::msg::UInt8MultiArray();
        
        //Vypocet rychlosti
        algorithms::WheelSpeed wheel_speed = kinematics_.inverse(cmd_vel_);
        //Prepocet na hodnoty do driveru
        auto w_l = static_cast<uint8_t>((255-127)/max_speed*wheel_speed.l + 127);
        auto w_r = static_cast<uint8_t>((255-127)/max_speed*wheel_speed.r + 127);

        msg.data={w_l, w_r};

        publisher_->publish(msg);
        //RCLCPP_INFO(this->get_logger(), "Sending speeeed %d %d",i,i);
    }

    void MotorNode::encoder_callback(const std_msgs::msg::UInt32MultiArray::SharedPtr msg) {
        encoders_ = {msg->data[0], msg->data[1]};
        //RCLCPP_INFO(this->get_logger(), "Receiving encoder data %u %u",msg->data[0],msg->data[1]);
    }

    void MotorNode::cmd_vel_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        
        //Nastavi cmd_vel_ na prijatou hodnotu
        cmd_vel_ = {msg->data[0], msg->data[1]}; 
    }
}
