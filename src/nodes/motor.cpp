#include "nodes/motor.hpp"

#include <cmath>
#include <cstdint>
#include <cstdlib>

namespace nodes {
    int i = 0;

    namespace {
        int64_t calculate_encoder_delta(uint32_t previous, uint32_t current) {
            constexpr int64_t encoder_wrap = int64_t{1} << 32;
            constexpr int64_t half_encoder_wrap = int64_t{1} << 31;

            int64_t delta = static_cast<int64_t>(current) - static_cast<int64_t>(previous);
            if (delta > half_encoder_wrap) {
                delta -= encoder_wrap;
            } else if (delta < -half_encoder_wrap) {
                delta += encoder_wrap;
            }

            return delta;
        }
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
        if (msg->data.size() < 2) {
            RCLCPP_WARN(this->get_logger(), "Encoder message has %zu values, expected at least 2", msg->data.size());
            return;
        }

        uint32_t new_l = msg->data[0];
        uint32_t new_r = msg->data[1];

        if (encoders_initialized_) {
            int64_t delta_l = calculate_encoder_delta(encoders_.l, new_l);
            int64_t delta_r = calculate_encoder_delta(encoders_.r, new_r);

            double meters_per_tick = (2.0 * M_PI * wheel_radius) / static_cast<double>(TPR);
            double distance_l = std::abs(delta_l) * meters_per_tick;
            double distance_r = std::abs(delta_r) * meters_per_tick;

            encoder_distance_total_ += static_cast<float>((distance_l + distance_r) / 2.0);
        } else {
            encoders_initialized_ = true;
        }

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
