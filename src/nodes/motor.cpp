#include "nodes/motor.hpp"
#include <cstdint>

namespace nodes {

        constexpr float dt = 2e-3;
        constexpr float max_linear_accel = 0.8;
        constexpr float max_angluar_accel = 9;

        MotorNode::MotorNode() : rclcpp::Node("motor_node"), kinematics_(algorithms::wheel_radius,algorithms::wheel_base,algorithms::TPR), 
        cmd_vel_({0,0}),
        current_vel_({0,0}), 
        encoders_({0,0}), 
        act_coords_({0,0})
        {
            publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(Topic::set_motor_speeds,1); 
            coords_publisher_ = create_publisher<std_msgs::msg::Float32MultiArray>(Topic::coords, rclcpp::SensorDataQoS());

            subscriber_ = this->create_subscription<std_msgs::msg::UInt32MultiArray>(
                            Topic::encoders,
                            rclcpp::SensorDataQoS(),
                            std::bind(&MotorNode::encoder_callback, this, std::placeholders::_1)
                        );

            subscriber_cmd_vel_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
                Topic::cmd_vel,
                rclcpp::SensorDataQoS(),
                std::bind(&MotorNode::cmd_vel_callback, this, std::placeholders::_1));

            timer_ = this->create_wall_timer(2ms,std::bind(&MotorNode::set_speed_callback, this));
            coords_pub_timer_ = this->create_wall_timer(10ms,std::bind(&MotorNode::publish_coords_cb, this));
        }

    void MotorNode::set_speed_callback() {

        auto msg = std_msgs::msg::UInt8MultiArray();

        //Hard stop
        /*
        if (cmd_vel_.v == 0 && cmd_vel_.w == 0){ 
            msg.data={127, 127};
            publisher_->publish(msg);
            return;
        }
        */
        // --- Acceleration filter ---
        auto ramp = [](float current, float target, float max_delta) -> float {
            float diff = target - current;
            if (diff >  max_delta) return current + max_delta;
            if (diff < -max_delta) return current - max_delta;
            return target;
        };

        const float max_lin_step = max_linear_accel * dt;
        const float max_ang_step = max_angluar_accel * dt;

        current_vel_.v = ramp(current_vel_.v,  cmd_vel_.v,  max_lin_step);
        current_vel_.w = ramp(current_vel_.w, cmd_vel_.w, max_ang_step);
        // ---------------------------

        // Use current_vel_ instead of cmd_vel_
        algorithms::WheelSpeed wheel_speed = kinematics_.inverse(current_vel_);
        
        
        //Prepocet na hodnoty do driveru
        auto w_l = static_cast<uint8_t>((255-127)/algorithms::max_speed*wheel_speed.l + 127);
        auto w_r = static_cast<uint8_t>((255-127)/algorithms::max_speed*wheel_speed.r + 127);

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
