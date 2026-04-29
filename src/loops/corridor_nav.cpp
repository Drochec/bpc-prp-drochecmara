#include <corridor_nav.hpp>
#include <math.h>
#include <sys/stat.h>

#include "imu_node.hpp"
#include "lidar_node.hpp"

// Constants for encoder distance calculation
constexpr double WHEEL_RADIUS = 68.55e-3;  // meters
constexpr int TPR = 585;  // Ticks per revolution

namespace loops {

    float calculate_distance_from_encoders(
        const algorithms::Encoders& start_encoders,
        const algorithms::Encoders& current_encoders
    ) {
        // Calculate delta in ticks for each wheel
        uint32_t delta_left = current_encoders.l - start_encoders.l;
        uint32_t delta_right = current_encoders.r - start_encoders.r;
        
        // Average of both wheels
        double avg_delta_ticks = (delta_left + delta_right) / 2.0;
        
        // Convert ticks to distance: (ticks / TPR) * circumference
        // circumference = 2 * pi * radius
        float distance = (avg_delta_ticks / TPR) * 2.0 * M_PI * WHEEL_RADIUS;
        
        return distance;
    }


    void CorridorNav::state_machine() {
        
        if (state_ != last_state_) {
            RCLCPP_INFO(get_logger(), "State: %u",  static_cast<unsigned int>(state_));
            last_state_ = state_;
        }

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

                cmd_vel_.w = pid_yaw_.step(error_yaw);


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

            case corridor_state::INTERSECTION:
                // Store sensor readings for turn direction decision
                //turn left
                if (lidar_vals_.left > wall_threshold) {
                    next_turn_direction_state_ = corridor_state::TURNING;
                    set_yaw_ = yaw_estimate_ + M_PI/2;
                }
                //turn right
                else if(lidar_vals_.right > wall_threshold) {
                    next_turn_direction_state_ = corridor_state::TURNING;
                    set_yaw_ = yaw_estimate_ - M_PI/2;
                }
                else if (lidar_vals_.front > wall_threshold) {
                    exiting_corridor_ = true;
                    next_turn_direction_state_ = corridor_state::CORRIDOR_FOLLOWING;
                }
                //turn back
                else {
                    next_turn_direction_state_ = corridor_state::TURNING;
                    set_yaw_ = yaw_estimate_ + M_PI;
                }

                // Save encoder position and reset distance tracker
                encoders_at_intersection_start_ = encoders_;
                distance_traveled_at_intersection_ = 0.0f;
                
                // Transition to advance state to move 15cm before committing to turn
                pid_yaw_.reset();
                state_ = corridor_state::INTERSECTION_ADVANCE;
                break;

            case corridor_state::INTERSECTION_ADVANCE:
                // Move forward while tracking distance via encoders
                cmd_vel_.v = forward_speed_corridor;
                cmd_vel_.w = 0.0;  // Go straight, no rotation
                
                // Calculate distance traveled since intersection detection
                distance_traveled_at_intersection_ = calculate_distance_from_encoders(
                    encoders_at_intersection_start_,
                    encoders_
                );
                
                // Once 15cm traveled, proceed to turn based on decision made above
                if (distance_traveled_at_intersection_ >= intersection_advance_distance) {
                    cmd_vel_ = {0.0, 0};
                    state_ = next_turn_direction_state_;
                    
                    if (next_turn_direction_state_ == corridor_state::TURNING) {
                        pid_yaw_.reset();
                    }
                }
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
                cmd_vel_.w = pid_yaw_.step(error_yaw);
                cmd_vel_.v = 0.2;

                if (abs(error_yaw) <= 0.1) {

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

    void CorridorNav::set_state_callback(std_msgs::msg::UInt8::SharedPtr msg) {
        if (msg->data == 0) {
            cmd_vel_ = {0, 0};
            state_ = corridor_state::CALIBRATION;
        }
    }
    
    void CorridorNav::encoder_callback(std_msgs::msg::UInt32MultiArray::SharedPtr msg) {
        encoders_.l = msg->data[0];
        encoders_.r = msg->data[1];
    }

}
