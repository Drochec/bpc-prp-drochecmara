#include <corridor_bang.hpp>
#include <math.h>
#include <sys/stat.h>

#include "imu_node.hpp"
#include "lidar_node.hpp"

namespace loops {




    void CorridorBang::publish_cmd_vel(){
        auto msg = std_msgs::msg::Float32MultiArray();
        

        msg.data = {cmd_vel_.v,cmd_vel_.w};

        publisher_cmd_vel_->publish(msg);

    }

    void CorridorBang::corridor_est_discrete_callback(std_msgs::msg::Float32MultiArray::SharedPtr msg) {

        lidar_vals_ = {msg->data[0],msg->data[1],msg->data[2],msg->data[3]};

    }

    void CorridorBang::yaw_est_callback(std_msgs::msg::Float32::SharedPtr msg) {

        yaw_estimate_ = msg->data;
    }

    void CorridorBang::set_state_callback(std_msgs::msg::UInt8::SharedPtr msg) {
        if (msg->data == 0) {
            cmd_vel_ = {0, 0};
            state_ = corridor_state::CALIBRATION;
        }
    }

    
    double set_yaw = 0;

    void CorridorBang::state_machine_driving() {
        RCLCPP_INFO(get_logger(), "State: %d",  (int)state_);


        error_walls_ = lidar_vals_.left - lidar_vals_.right;
        //auto error_yaw = set_yaw - yaw_estimate_;

        switch (state_) {
            case corridor_state::CALIBRATION:
                if (isnan(yaw_estimate_)) {
                    break;
                }
                else {
                    state_ = corridor_state::CORRIDOR_FOLLOWING;
                }

            case corridor_state::CORRIDOR_FOLLOWING:
                //Dead-end check
                /*
                if (lidar_vals_.front <= 0.25) {
                    cmd_vel_ = {0, 0};
                    //Check volny smer
                    if (lidar_vals_.left >= 0.5) {
                        //set yaw to + pi/2
                        set_yaw = yaw_estimate_ + M_PI/2;
                    }
                    else if(lidar_vals_.right >= 0.5) {
                        //set yaw to - pi/2
                        set_yaw = yaw_estimate_ - M_PI/2;
                    }
                    else {
                        set_yaw = yaw_estimate_ + M_PI;
                    }
                    state_ = corridor_state::TURNING;
                    pid_yaw_.reset();
                    break;
                }
                */
                //Corridor following
                cmd_vel_.v = forward_speed_corridor;

                if (abs(error_walls_) < 1e-3) {
                    error_walls_ = 0;
                }

                //cmd_vel_.w = pid_walls_.step(error_walls_,10e-3);

                cmd_vel_.w = K * error_walls_;
                
                //RCLCPP_INFO(this->get_logger(),"L: %.3f R: %.3f error: %.3f w: %.3f",lidar_vals_.left, lidar_vals_.right, error, cmd_vel_.w);

                // Keep centered using P/PID based on side distances
                // If front is blocked and one side is open → switch to TURNING
                break;

                /*
            case corridor_state::TURNING:
                // Use IMU to track rotation
                // Rotate until yaw changes by ±90°
                // Then return to CORRIDOR_FOLLOWING

                RCLCPP_INFO(get_logger(), "Error yaw: %lf",  error_yaw);
                cmd_vel_.w = pid_yaw_.step(error_yaw,30e-3);

                if (abs(error_yaw) <= 0.01) {
                    cmd_vel_ = {0, 0};
                    state_ = corridor_state::CORRIDOR_FOLLOWING;
                }

                break;
                */
        }





    }

}
