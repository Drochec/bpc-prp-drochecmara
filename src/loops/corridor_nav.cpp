#include "corridor_nav.hpp"
#include <math.h>
#include <sys/stat.h>

#include "imu_node.hpp"
#include "lidar_node.hpp"

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
        float distance = (avg_delta_ticks / encoder_ticks_per_revolution) * 2.0 * M_PI * encoder_wheel_radius;
        
        return distance;
    }


    void CorridorNav::state_machine() {

    if (state_ != last_state_) {
        RCLCPP_INFO(get_logger(), "State: %u", static_cast<unsigned int>(state_));
        last_state_ = state_;
    }

    // --- Sensor errors ---
    auto error_lidar = lidar_vals_.left - lidar_vals_.right;
    auto error_yaw = normalize_angle(set_yaw_ - yaw_estimate_);

    // --- Open direction detection ---
    bool left_open  = lidar_vals_.left  > wall_threshold;
    bool right_open = lidar_vals_.right > wall_threshold;
    bool front_open = lidar_vals_.front > wall_threshold;

    int open_directions = (left_open ? 1 : 0) +
                          (right_open ? 1 : 0) +
                          (front_open ? 1 : 0);

    switch (state_) {

        // ---------------- CALIBRATION ----------------
        case corridor_state::CALIBRATION:
            if (!std::isnan(yaw_estimate_)) {
                set_yaw_ = yaw_estimate_;
                state_ = corridor_state::CORRIDOR_FOLLOWING;
            }
            break;

        // ---------------- CORRIDOR FOLLOWING ----------------
        case corridor_state::CORRIDOR_FOLLOWING:

            // ✅ NEW: detect ANY intersection (not just front-blocked)
            if (open_directions >= 2) {

                intersection_start_encoders_ = current_encoders_;

                // --- Decide preferred turn ---
                if (left_open) {
                    intersection_turn_direction_ = +1;
                } else if (right_open) {
                    intersection_turn_direction_ = -1;
                } else {
                    intersection_turn_direction_ = 0; // straight
                }

                pid_yaw_.reset();

                state_ = corridor_state::INTERSECTION_APPROACH;
                break;
            }

            // Safety stop
            if (!front_open && lidar_vals_.front <= front_stop) {
                cmd_vel_ = {0, 0};
                state_ = corridor_state::INTERSECTION;
                break;
            }

            // Normal corridor following
            cmd_vel_.v = forward_speed_corridor;
            cmd_vel_.w = pid_yaw_.step(error_yaw, 20e-3);
            break;

        // ---------------- INTERSECTION APPROACH ----------------
        case corridor_state::INTERSECTION_APPROACH:

            cmd_vel_.v = forward_speed_corridor;
            cmd_vel_.w = 0.0;

            if (calculate_distance_from_encoders(
                    intersection_start_encoders_,
                    current_encoders_) >= 0.15) {

                cmd_vel_ = {0.0, 0.0};
                state_ = corridor_state::INTERSECTION;
            }
            break;

        // ---------------- INTERSECTION DECISION ----------------
        case corridor_state::INTERSECTION:

            if (intersection_turn_direction_ > 0) {
                set_yaw_ = yaw_estimate_ + M_PI / 2;   // LEFT
            }
            else if (intersection_turn_direction_ < 0) {
                set_yaw_ = yaw_estimate_ - M_PI / 2;   // RIGHT
            }
            else {
                // Go straight
                state_ = corridor_state::EXIT_INTERSECTION;
                break;
            }

            pid_yaw_.reset();
            state_ = corridor_state::TURNING;
            break;

        // ---------------- TURNING ----------------
        case corridor_state::TURNING:

            cmd_vel_.v = 0.0;
            cmd_vel_.w = pid_yaw_.step(error_yaw, 20e-3);

            if (std::abs(error_yaw) < 0.1) {
                cmd_vel_ = {0.0, 0.0};
                set_yaw_ = yaw_estimate_;
                pid_yaw_.reset();
                state_ = corridor_state::EXIT_INTERSECTION;
            }
            break;

        // ---------------- EXIT INTERSECTION ----------------
        case corridor_state::EXIT_INTERSECTION:

            cmd_vel_.v = forward_speed_corridor;
            cmd_vel_.w = 0.0;

            // Once we see corridor walls again → back to normal
            if (!left_open || !right_open) {
                set_yaw_ = yaw_estimate_;
                pid_yaw_.reset();
                state_ = corridor_state::CORRIDOR_FOLLOWING;
            }
            break;

        // ---------------- RESET ----------------
        case corridor_state::RESET:
            cmd_vel_ = {0, 0};
            set_yaw_ = 0;
            pid_yaw_.reset();
            pid_centering_.reset();
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
            return;
        }

        response->success = true;
        response->message = "Commanded state received";
    }

    void CorridorNav::publish_cmd_vel(){
        auto msg = std_msgs::msg::Float32MultiArray();
        msg.data = {cmd_vel_.v, cmd_vel_.w};
        publisher_cmd_vel_->publish(msg);
    }

    void CorridorNav::range_est_callback(std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        lidar_vals_ = {msg->data[0], msg->data[1], msg->data[2], msg->data[3]};
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

    void CorridorNav::encoder_distance_callback(std_msgs::msg::Float32::SharedPtr msg) {
        encoder_distance_total_ = msg->data;
    }

}
