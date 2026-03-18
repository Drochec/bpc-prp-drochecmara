#include <line_loop_bang.hpp>
#include <sys/stat.h>

#include "line.hpp"


namespace loops {
    void BangBang::publish_cmd_vel(){
        auto msg = std_msgs::msg::Float32MultiArray();


        msg.data = {cmd_vel_.v,cmd_vel_.w};

        publisher_cmd_vel_->publish(msg);
    }

    void BangBang::line_est_discrete_callback(std_msgs::msg::UInt8::SharedPtr msg) {
        DiscreteLinePose pose{msg->data};

        switch (pose) {
            case DiscreteLinePose::LineOnLeft:
                cmd_vel_.w = turn_speed;
                break;
            case DiscreteLinePose::LineOnRight:
                cmd_vel_.w = -turn_speed;
                break;
            case DiscreteLinePose::LineBoth:
                cmd_vel_.w = 0;
                break;
            case DiscreteLinePose::LineNone:
                cmd_vel_.w = turn_speed;
        }
    }
}
