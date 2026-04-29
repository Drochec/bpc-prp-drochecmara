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
        // Compute distance increment from previous encoder readings and publish total distance
        uint32_t new_l = msg->data[0];
        uint32_t new_r = msg->data[1];

        uint32_t delta_l = new_l - encoders_.l;
        uint32_t delta_r = new_r - encoders_.r;

        double avg_delta_ticks = (static_cast<double>(delta_l) + static_cast<double>(delta_r)) / 2.0;

        // Convert ticks to distance: (ticks / TPR) * circumference
        double distance = (avg_delta_ticks / static_cast<double>(TPR)) * 2.0 * M_PI * wheel_radius;

        encoder_distance_total_ += static_cast<float>(distance);

        // Update stored encoder readings
        encoders_.l = new_l;
        encoders_.r = new_r;

        // Publish total distance traveled
        auto out = std_msgs::msg::Float32();
        out.data = encoder_distance_total_;
        if (publisher_encoder_distance_) publisher_encoder_distance_->publish(out);

        //RCLCPP_INFO(this->get_logger(), "Receiving encoder data %u %u",msg->data[0],msg->data[1]);
    }

    void MotorNode::cmd_vel_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        
        //Nastavi cmd_vel_ na prijatou hodnotu
        cmd_vel_ = {msg->data[0], msg->data[1]}; 
    }
}
