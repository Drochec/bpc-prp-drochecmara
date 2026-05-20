#include "corridor_nav.hpp"

namespace loops{


    void CorridorNav::state_machine() {
        //RCLCPP_INFO(get_logger(),"Distance driven: %f",pose_.x);
        //if (line_detection_ > 0) RCLCPP_INFO(get_logger(),"Line detected: %u",line_detection_);

        if (std::isnan(lidar_vals_.back)){
            RCLCPP_INFO(get_logger(),"Lidar data lost signal received stopping!");
            cmd_vel_ = {0,0};
            return;
        }
        
        if (state_ != last_state_) {
            RCLCPP_INFO(get_logger(), "State: %u",  static_cast<unsigned int>(state_));
            last_state_ = state_;
        }

        float error_lidar = lidar_vals_.left - lidar_vals_.right;

        auto error_yaw = set_yaw_ - yaw_estimate_;

        //Bias the yaw error if too close to a wall
        float error_bias_left = bias_gain*(std::max(-lidar_vals_.left+centering_treshold,0.0F)) ;
        error_bias_left += 3*(std::max(-intersection_vals_.left+centering_treshold, 0.0F));
        float error_bias_right = bias_gain*(std::max(-lidar_vals_.right+centering_treshold,0.0F));
        error_bias_right += 3*(std::max(-intersection_vals_.right+centering_treshold,0.0F));
        //RCLCPP_INFO(get_logger(),"Yaw Error: %lf", error_yaw);

        //RCLCPP_INFO(get_logger(), "Right B: %lf Left B: %lf",error_bias_right,error_bias_left);

        //Don't using biasing when turning using gyro
        if (state_ != corridor_state::TURNING)
            error_yaw = error_yaw - error_bias_left + error_bias_right;

        switch (state_) {

            case corridor_state::CALIBRATION:
                if (isnan(yaw_estimate_)) {
                    break;
                }
                else {
                    kinematics_.reset_pose(encoders_);
                    pose_ = {0,0,0};
                    state_ = corridor_state::CORRIDOR_FOLLOWING;
                    break;
                }

            case corridor_state::CORRIDOR_FOLLOWING:

                if ((lidar_vals_.front < wall)) {
                        cmd_vel_ = {0, 0};
                        state_ = corridor_state::INTERSECTION;
                        kinematics_.reset_pose(encoders_);
                        pose_ = {0,0,0};
                        RCLCPP_INFO(get_logger(),"Path blocked");
                        break;
                    }
                if (lidar_vals_.front >= free_space){        
                    if (intersection_vals_.left > intersection_threshold || intersection_vals_.right > intersection_threshold) {
                        state_ = corridor_state::INTERSECTION_ADVANCE;
                        //last_coords_ = coords_;
                        kinematics_.reset_pose(encoders_);
                        pose_ = {0,0,0};
                        RCLCPP_INFO(get_logger(),"Lidar reports intersection ahead");
                        break;
                    }
                }
 
                //Corridor following
                cmd_vel_.v = forward_speed_corridor;
                cmd_vel_.w = pid_yaw_.step(error_yaw, dt);

                break;

            case corridor_state::INTERSECTION:

                valid_paths_ = check_valid_paths();
                
                //If only one valid paths (not counting back) -> not an intersection
                if (std::count(valid_paths_.begin(),valid_paths_.end(), true) <= 1) {

                    RCLCPP_INFO(get_logger(), "Only one valid path, skipping tags");

                    set_yaw_ = yaw_estimate_ + yaw_from_valid_path(valid_paths_);

                    if (set_yaw_ == yaw_estimate_) {//Continuing straight
                        state_ = corridor_state::CORRIDOR_FOLLOWING;
                        kinematics_.reset_pose(encoders_);
                        pose_ = {0,0,0};
                        break;
                    }

                    state_ = corridor_state::TURNING;
                    RCLCPP_INFO(get_logger(),"Turning");
                    kinematics_.reset_pose(encoders_);
                    pose_ = {0,0,0};
                    turn_start_time_ = now();
                    break;
                }

                if (handle_direction(treasure_)) break;

                if (handle_direction(exit_)) break;

                set_yaw_= yaw_estimate_ + yaw_from_valid_path(valid_paths_);

                if (set_yaw_ == yaw_estimate_) { //Should not happen but in case
                        state_ = corridor_state::CORRIDOR_FOLLOWING;
                        kinematics_.reset_pose(encoders_);
                        pose_ = {0,0,0};
                        break;
                }

                state_ = corridor_state::TURNING;
                RCLCPP_INFO(get_logger(),"Turning");
                kinematics_.reset_pose(encoders_);
                pose_ = {0,0,0};
                turn_start_time_ = now();
                break;
                

            case corridor_state::INTERSECTION_ADVANCE:
                //RCLCPP_INFO(get_logger(),"Driven: %f",pose_.x);
                // Move forward while tracking distance via encoders
                cmd_vel_.v = forward_speed_corridor;
                cmd_vel_.w = pid_yaw_.step(error_yaw, dt);  // Go straight, no rotation
                
                // Once 15cm traveled or about to hit a wall, proceed to turn based on decision made above

                //Reset driven distance when going over a black line = intersection edge
                if (line_detection_ == 3) {
                    kinematics_.reset_pose(encoders_);
                    pose_ = {0,0,0};
                    RCLCPP_INFO(get_logger(),"Line detected - Distance Reset");
                    break;
                }

                if (lidar_vals_.front < free_space){
                    if (lidar_vals_.front <= wall) {
                        cmd_vel_ = {0.0, 0};
                        state_ = corridor_state::INTERSECTION;
                        send_reset_yaw();
                        RCLCPP_INFO(get_logger(),"Intersection entered");
                    }
                } 
                else if (std::abs(pose_.x) >= intersection_advance_distance) { 

                        RCLCPP_INFO(get_logger(),"Intersection entered based on distance driven: %f",pose_.x);
                        cmd_vel_ = {0.0, 0};
                        state_ = corridor_state::INTERSECTION;
                        send_reset_yaw();
                }
                break;
                

            case corridor_state::EXIT_INTERSECTION:
                cmd_vel_.v = exit_speed; 
                cmd_vel_.w = pid_yaw_.step(error_yaw, dt);

                //RCLCPP_INFO(get_logger(), "Distance driven: %lf", pose_.x);

                // Condition: walls detected or moved 25 cm
                if ((lidar_vals_.left < free_space && lidar_vals_.right < free_space && pose_.x >= 15e-2) || 
                    (pose_.x >= 30e-2)) {

                    kinematics_.reset_pose(encoders_);
                    pose_ = {0,0,0};
                    state_ = corridor_state::CORRIDOR_FOLLOWING;
                    RCLCPP_INFO(get_logger(),"Exit complete");
                
                }

                break;

            case corridor_state::TURNING:
                // Use IMU to track rotation
                // Rotate until yaw changes by ±90°
                // Then return to CORRIDOR_FOLLOWING

                //RCLCPP_INFO(get_logger(), "Error yaw: %lf",  error_yaw);Y
                cmd_vel_.w = pid_yaw_turn_.step(error_yaw, dt);
                

                if ((abs(error_yaw) <= 0.02) || ((now() - turn_start_time_).seconds() >= 2)) {

                    cmd_vel_ = {0.0, 0};
                    pid_yaw_.reset();
                    pid_yaw_turn_.reset();
                    set_yaw_ = 0;
                    send_reset_yaw();
                    //last_coords_ = coords_;
                    state_ = corridor_state::EXIT_INTERSECTION;
                    RCLCPP_INFO(get_logger(),"Turn complete, exiting");
                }

                break;

            case corridor_state::RESET:

                cmd_vel_ = {0, 0};
                set_yaw_ = 0;
                send_reset_yaw();
                pid_yaw_.reset();
                pid_yaw_turn_.reset();
                exit_ = algorithms::ArucoID::NONE;
                treasure_ = algorithms::ArucoID::NONE;
                heading_to_exit_ = false;
                //exiting_corridor
                state_ = corridor_state::WAIT;

                break;

        }

    }
}