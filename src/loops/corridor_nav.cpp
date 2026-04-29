#include <corridor_nav.hpp>
#include <math.h>
#include <sys/stat.h>

#include "imu_node.hpp"
#include "lidar_node.hpp"

namespace loops {

    static float normalize_angle(float angle) {
        while (angle > M_PI) angle -= 2.0f * M_PI;
        while (angle <= -M_PI) angle += 2.0f * M_PI;
        return angle;
    }

    static float encoder_distance_m(const algorithms::Encoders &start,
                                   const algorithms::Encoders &current) {
        int64_t raw_l = int64_t(current.l) - int64_t(start.l);
        if (raw_l > INT32_MAX) raw_l -= 4294967296ll;
        else if (raw_l < INT32_MIN) raw_l += 4294967296ll;
        int32_t delta_l = static_cast<int32_t>(raw_l);

        int64_t raw_r = int64_t(current.r) - int64_t(start.r);
        if (raw_r > INT32_MAX) raw_r -= 4294967296ll;
        else if (raw_r < INT32_MIN) raw_r += 4294967296ll;
        int32_t delta_r = static_cast<int32_t>(raw_r);

        float d_L = float(delta_l) * M_PI * loops::encoder_wheel_radius / loops::encoder_ticks_per_revolution;
        float d_R = float(delta_r) * M_PI * loops::encoder_wheel_radius / loops::encoder_ticks_per_revolution;
        return (d_L + d_R) * 0.5f;
    }

    void CorridorNav::state_machine() {
        
        if (state_ != last_state_) {
            RCLCPP_INFO(get_logger(), "State: %u",  static_cast<unsigned int>(state_));
            last_state_ = state_;
        }

        auto error_lidar = lidar_vals_.left - lidar_vals_.right;
        //float error_lidar = (lidar_vals_.left - lidar_vals_.right) / (lidar_vals_.left + lidar_vals_.right);
        auto error_yaw = normalize_angle(set_yaw_ - yaw_estimate_);
        bool side_open = false;
        bool front_blocked = false;

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


                // Intersection detection: require a blocked front and an open side.
                front_blocked = lidar_vals_.front <= wall_threshold;
                side_open = lidar_vals_.left > wall_threshold || lidar_vals_.right > wall_threshold;

                if (front_blocked && side_open) {
                    intersection_start_encoders_ = current_encoders_;
                    intersection_turn_direction_ = lidar_vals_.left > wall_threshold ? +1 : -1;
                    pid_yaw_.reset();
                    cmd_vel_.v = forward_speed_corridor;
                    cmd_vel_.w = 0.0;
                    state_ = corridor_state::INTERSECTION_APPROACH;
                    break;
                }

                if (lidar_vals_.front <= front_stop) {
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

                cmd_vel_.w = pid_yaw_.step(error_yaw,20e-3);


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
                cmd_vel_.w = pid_centering_.step(error_lidar);

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

            case corridor_state::INTERSECTION_APPROACH:
                if (!encoders_ready_) {
                    state_ = corridor_state::INTERSECTION;
                    break;
                }

                if (lidar_vals_.front <= front_stop) {
                    state_ = corridor_state::INTERSECTION;
                    break;
                }

                if (encoder_distance_m(intersection_start_encoders_, current_encoders_) >=
                    intersection_delay_distance) {
                    cmd_vel_ = {0.0, 0.0};
                    state_ = corridor_state::INTERSECTION;
                    break;
                }

                cmd_vel_.v = forward_speed_corridor;
                cmd_vel_.w = 0.0;
                break;

            case corridor_state::INTERSECTION:
                if (intersection_turn_direction_ > 0) {
                    set_yaw_ = yaw_estimate_ + M_PI/2;
                }
                else if (intersection_turn_direction_ < 0) {
                    set_yaw_ = yaw_estimate_ - M_PI/2;
                }
                else if (lidar_vals_.front > wall_threshold) {
                    exiting_corridor_ = true;
                    state_ = corridor_state::CORRIDOR_FOLLOWING;
                    break;
                }
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
                // Rotate in place until yaw changes by ±90°
                // Then return to EXIT_INTERSECTION

                RCLCPP_INFO(get_logger(), "Error yaw: %lf",  error_yaw);
                cmd_vel_.w = pid_yaw_.step(error_yaw, 20e-3);
                cmd_vel_.v = 0.0;

                if (std::abs(error_yaw) <= 0.1) {

                    cmd_vel_ = {0.0, 0};
                    pid_yaw_.reset();
                    set_yaw_ = yaw_estimate_;
                    state_ = corridor_state::EXIT_INTERSECTION;
                }

                break;

            case corridor_state::RESET:

                cmd_vel_ = {0, 0};
                set_yaw_ = 0;
                pid_yaw_.reset();
                pid_centering_.reset();
                //exiting_corridor
                state_ = corridor_state::WAIT;

                break;

        }



    }

    void CorridorNav::send_calibrate_trigger(){

        auto request = std::make_shared<prp_project::srv::CalibrateTrigger::Request>();

        calibrate_client_->async_send_request(request);
    }
    
    void CorridorNav::send_reset_yaw(){

        auto request = std::make_shared<prp_project::srv::ResetYawTrigger::Request>();

        reset_yaw_client_->async_send_request(request);
    }

    void CorridorNav::button_cmd_handle(
            const std::shared_ptr<prp_project::srv::ButtonCmd::Request> request,
            std::shared_ptr<prp_project::srv::ButtonCmd::Response> response
        ) {
        
        auto received_state = request->command;

        if (received_state == "START") {
            send_calibrate_trigger();
            state_ = corridor_state::CALIBRATION;
        }

        else if (received_state == "STOP") {
            state_ = corridor_state::RESET;
        }

        else {
            RCLCPP_WARN(get_logger(),"Unrecognized command from buttons received, ignoring...");
            response->success = false;
            response->message = "Unrecognized";
            return ;
        }

        response->success = true;
        response->message = "Commanded state received";
        
        }

    void CorridorNav::publish_cmd_vel(){
        auto msg = std_msgs::msg::Float32MultiArray();

        msg.data = {cmd_vel_.v,cmd_vel_.w};

        publisher_cmd_vel_->publish(msg);
    }

    void CorridorNav::range_est_callback(std_msgs::msg::Float32MultiArray::SharedPtr msg) {

        lidar_vals_ = {msg->data[0],msg->data[1],msg->data[2],msg->data[3]};

    }

    void CorridorNav::yaw_est_callback(std_msgs::msg::Float32::SharedPtr msg) {

        yaw_estimate_ = msg->data;
    }

    void CorridorNav::encoders_callback(std_msgs::msg::UInt32MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 2) {
            current_encoders_ = {msg->data[0], msg->data[1]};
            encoders_ready_ = true;
        }
    }

    void CorridorNav::set_state_callback(std_msgs::msg::UInt8::SharedPtr msg) {
        if (msg->data == 0) {
            cmd_vel_ = {0, 0};
            state_ = corridor_state::CALIBRATION;
        }
    }

}
