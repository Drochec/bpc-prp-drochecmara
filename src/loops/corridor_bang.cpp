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


    void CorridorBang::state_machine_driving() {
        RCLCPP_INFO(get_logger(), "State: %u",  static_cast<unsigned int>(state_));


        auto error_lidar = lidar_vals_.left - lidar_vals_.right;
        //float error_lidar = (lidar_vals_.left - lidar_vals_.right) / (lidar_vals_.left + lidar_vals_.right);
        auto error_yaw = set_yaw_ - yaw_estimate_;

        switch (state_) {
            case corridor_state::CALIBRATION:
                if (isnan(yaw_estimate_)) {
                    break;
                }
                else {
                    //set_yaw = yaw_estimate_;
                    state_ = corridor_state::CORRIDOR_FOLLOWING;
                }

            case corridor_state::CORRIDOR_FOLLOWING:


                //Dead-end check

                if (lidar_vals_.left > wall_threshold || lidar_vals_.right > wall_threshold) {

                    if (lidar_vals_.front < wall_threshold) {
                        if (lidar_vals_.front <= front_stop) {
                            cmd_vel_ = {0, 0};
                            //Check volny smer
                            state_ = corridor_state::INTERSECTION;
                            //pid_yaw_.reset();
                            break;
                        }
                    }

                    cmd_vel_ = {0, 0};
                    state_ = corridor_state::INTERSECTION;
                    break;
                }
                /*else if (lidar_vals_.left < 0.15 || lidar_vals_.right < 0.15) {
                    state_ = corridor_state::CENTERING;
                    break;
                }*/

                //Corridor following
                cmd_vel_.v = forward_speed_corridor;
                //cmd_vel_.v = 1.765 * lidar_vals_.front - 0.265;

                cmd_vel_.w = pid_yaw_.step(error_yaw,30e-3);


                //RCLCPP_INFO(this->get_logger(),"L: %.3f R: %.3f error: %.3f w: %.3f",lidar_vals_.left, lidar_vals_.right, error, cmd_vel_.w);

                // Keep centered using P/PID based on side distances
                // If front is blocked and one side is open → switch to TURNING
                break;

            case corridor_state::CENTERING:
                // Safety: if we lose walls → bail out
                if (!(lidar_vals_.left < wall_threshold &&
                      lidar_vals_.right < wall_threshold)) {
                    state_ = corridor_state::CORRIDOR_FOLLOWING;
                    break;
                      }

                // Aggressive controller
                cmd_vel_.w = pid_centering_.step(error_lidar, 30e-3);

                // Slow forward motion (important!)
                cmd_vel_.v = 0.15;

                // Exit condition (tighter than entry!)
                if (abs(error_lidar) < exit_centering_error) {
                    pid_centering_.reset();
                    pid_yaw_.reset();
                    set_yaw_ = yaw_estimate_;  // re-anchor heading
                    state_ = corridor_state::CORRIDOR_FOLLOWING;
                }

                break;

            case corridor_state::INTERSECTION:
                //turn left
                if (lidar_vals_.left > wall_threshold) {
                    set_yaw_ = yaw_estimate_ + M_PI/2;
                }
                //turn right
                else if(lidar_vals_.right > wall_threshold) {
                    set_yaw_ = yaw_estimate_ - M_PI/2;
                }
                else if (lidar_vals_.front > wall_threshold) {
                    exiting_corridor_ = true;
                    //TODO pomocou flag vypnut kontrolu stien,vytiahnut si z motorov hodnoty encoderov a vypnut flag, ked prejdeme 40 s robotom
                    state_ = corridor_state::CORRIDOR_FOLLOWING;
                }
                //turn back
                else {
                    set_yaw_ = yaw_estimate_ + M_PI;
                }

                pid_yaw_.reset();
                state_ = corridor_state::TURNING;
                break;

            case corridor_state::EXIT_INTERSECTION:
                cmd_vel_.v = forward_speed_corridor;   // move forward decisively
                cmd_vel_.w = 0.0;   // no turning

                // Condition: walls detected on ONE sides again
                if (lidar_vals_.left < front_stop ||
                    lidar_vals_.right < front_stop) {

                    // Optional: small stabilization
                    set_yaw_ = yaw_estimate_;
                    pid_yaw_.reset();

                    state_ = corridor_state::CORRIDOR_FOLLOWING;
                    }
                break;

            case corridor_state::TURNING:
                // Use IMU to track rotation
                // Rotate until yaw changes by ±90°
                // Then return to CORRIDOR_FOLLOWING

                RCLCPP_INFO(get_logger(), "Error yaw: %lf",  error_yaw);
                cmd_vel_.w = pid_yaw_.step(error_yaw,30e-3);
                cmd_vel_.v = 0.2;

                if (abs(error_yaw) <= 0.1) {

                    cmd_vel_ = {0.0, 0};
                    pid_yaw_.reset();
                    set_yaw_ = yaw_estimate_;
                    state_ = corridor_state::EXIT_INTERSECTION;
                }

                break;

        }





    }

}
