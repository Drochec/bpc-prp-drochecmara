#include "nodes/motor.hpp"
#include <cstdint>

namespace nodes {

    float constexpr max_speed = 19.5f; //rad/s
    double constexpr wheel_radius = 68.55e-3; //m
    double constexpr wheel_base = 130.00e-3; //m
    int constexpr TPR = 585; //Ticks per revolution


        MotorNode::MotorNode() : rclcpp::Node("motor_node"), kinematics_(wheel_radius,wheel_base,TPR), cmd_vel_({0,0}), encoders_({0,0}), act_coords_({0,0}) 
        {
            publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(Topic::set_motor_speeds, 10);
            coords_publisher_ = create_publisher<std_msgs::msg::Float32MultiArray>(Topic::coords, 10);

            subscriber_ = this->create_subscription<std_msgs::msg::UInt32MultiArray>(
                            Topic::encoders,
                            1,
                            std::bind(&MotorNode::encoder_callback, this, std::placeholders::_1)
                        );

            subscriber_cmd_vel_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
                Topic::cmd_vel,
                1,
                std::bind(&MotorNode::cmd_vel_callback, this, std::placeholders::_1));

            timer_ = this->create_wall_timer(5ms,std::bind(&MotorNode::set_speed_callback, this));
            coords_pub_timer_ = this->create_wall_timer(15ms,std::bind(&MotorNode::publish_coords_cb, this));
        }

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
                
        algorithms::Encoders encoders_new = {msg->data[0], msg->data[1]};
        
        act_coords_ = kinematics_.forward(encoders_new);

        //RCLCPP_INFO(this->get_logger(), "Receiving encoder data %u %u",msg->data[0],msg->data[1]);
    }

    void MotorNode::cmd_vel_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        
        //Nastavi cmd_vel_ na prijatou hodnotu
        cmd_vel_ = {msg->data[0], msg->data[1]}; 
    }

    void MotorNode::publish_coords_cb() {
        auto msg = std_msgs::msg::Float32MultiArray();

        msg.data = {act_coords_.x, act_coords_.y, act_coords_.fi};


        coords_publisher_->publish(msg);
    }
}
