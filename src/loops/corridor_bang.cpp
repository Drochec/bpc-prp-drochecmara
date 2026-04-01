#include <corridor_bang.hpp>
#include <sys/stat.h>

#include "lidar_node.hpp"

namespace loops {
    void CorridorBang::publish_cmd_vel(){
        auto msg = std_msgs::msg::Float32MultiArray();

        msg.data = {cmd_vel_.v,cmd_vel_.w};

        publisher_cmd_vel_->publish(msg);
    }

    void CorridorBang::corridor_est_discrete_callback(std_msgs::msg::Float32MultiArray::SharedPtr msg) {


        float K = 0.6;
        auto distance_front = msg->data[0];

        if (distance_front <= 0.25) {
            cmd_vel_ = {0,0};

        }

        else {
            cmd_vel_.v = forward_speed_corridor;

            auto distance_left = msg->data[2];
            auto distance_right = msg->data[3];

            auto error = distance_left - distance_right;
            if (abs(error) < 1e-3) {
                error = 0;
            }

            cmd_vel_.w = K * -error;
            RCLCPP_INFO(this->get_logger(),"L: %.3f R: %.3f error: %.3f w: %.3f",distance_left, distance_right, error, cmd_vel_.w);
        }
    }
}
