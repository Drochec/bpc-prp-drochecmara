#include "line_loop_pid.hpp"

namespace loops {
    void PidNode::publish_cmd_vel(){

        cmd_vel_.w = pid_.step(line_pose_,5e-3);

        auto msg = std_msgs::msg::Float32MultiArray();

        msg.data = {cmd_vel_.v,cmd_vel_.w};

        publisher_cmd_vel_->publish(msg);
    }

    void PidNode::line_est_callback(std_msgs::msg::Float32::SharedPtr msg) {

        line_pose_ = msg->data;

    }
}
