#include "loops/corridor_nav.hpp"

namespace loops {


constexpr float forward_speed_corridor = 0.1;
constexpr float free_space = 0.43;
constexpr float intersection_threshold = 0.53;
constexpr float wall = 0.23;
constexpr float wall_spacing = 0.15;
constexpr float exit_centering_error = 0.05;
constexpr float centering_treshold = 0.20;
constexpr float intersection_advance_distance = 0.25;  // 15cm in meters
constexpr float wall_avoidance_threshold = 0.18;
constexpr float wall_avoidance_max_yaw_error = 0.35;
constexpr float bias_gain = 6;

    void CorridorNav::state_machine() {
        
        if (state_ != last_state_) {
            RCLCPP_INFO(get_logger(), "State: %u",  static_cast<unsigned int>(state_));
            last_state_ = state_;
        }

        float error_lidar = lidar_vals_.left - lidar_vals_.right;


        auto error_yaw = set_yaw_ - yaw_estimate_;

        //Bias the yaw error if too close to a wall
        float error_bias_left = bias_gain*(std::max(-lidar_vals_.left+centering_treshold,0.0F)) ;
        float error_bias_right = bias_gain*(std::max(-lidar_vals_.right+centering_treshold,0.0F));
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
                    last_encoders_ = encoders_;
                    state_ = corridor_state::CORRIDOR_FOLLOWING;
                }

            case corridor_state::CORRIDOR_FOLLOWING:

                if ((lidar_vals_.front < wall)) {
                        cmd_vel_ = {0, 0};
                        state_ = corridor_state::INTERSECTION_ADVANCE;
                        last_encoders_ = encoders_;
                        RCLCPP_INFO(get_logger(),"Path blocked");
                        break;
                    }
                
                if (lidar_vals_.front >= free_space){        
                    if (intersection_vals_.left > intersection_threshold || intersection_vals_.right > intersection_threshold) {
                        state_ = corridor_state::INTERSECTION_ADVANCE;
                        //last_coords_ = coords_;
                        last_encoders_ = encoders_;
                        RCLCPP_INFO(get_logger(),"Lidar reports intersection ahead");
                        break;
                    }
                }
 
                //Corridor following
                cmd_vel_.v = forward_speed_corridor;
                cmd_vel_.w = pid_yaw_.step(error_yaw, dt);

                break;


            case corridor_state::INTERSECTION:

                // Check intersections for availible paths
                RCLCPP_INFO(get_logger(),"Checking for valid paths");
                RCLCPP_INFO(get_logger(),"lidar: %f, %f, %f, %f",lidar_vals_.front,lidar_vals_.back,lidar_vals_.left,lidar_vals_.right);
                RCLCPP_INFO(get_logger(),"intersection: %f, %f",intersection_vals_.left,intersection_vals_.right);

                //turn_set_ = false;
                if (handle_direction(treasure_)) break;
                if (handle_direction(exit_)) break;

                // Turn right
                if(lidar_vals_.right > intersection_threshold) {
                    set_yaw_ = yaw_estimate_ - M_PI/2;
                    RCLCPP_INFO(get_logger(),"Going right");
                }

                // Space in front -> continue
                else if (lidar_vals_.front > wall){
                    state_ = corridor_state::CORRIDOR_FOLLOWING;
                    set_yaw_ = yaw_estimate_;
                    last_encoders_ = encoders_;
                    RCLCPP_INFO(get_logger(),"Continuing forward");
                    break;
                }

                //turn left
                else if (lidar_vals_.left > intersection_threshold){
                    set_yaw_ = yaw_estimate_ + M_PI/2;
                    RCLCPP_INFO(get_logger(),"Going left");
                }
                 else {
                
                    set_yaw_ = yaw_estimate_ + M_PI;
                    RCLCPP_INFO(get_logger(),"Dead-end, turning back");
                }

                state_ = corridor_state::TURNING;
                RCLCPP_INFO(get_logger(),"Turning");
                break;
                

            case corridor_state::INTERSECTION_ADVANCE:
                //RCLCPP_INFO(get_logger(),"%f",act_coords_.x);
                // Move forward while tracking distance via encoders
                cmd_vel_.v = forward_speed_corridor;
                cmd_vel_.w = pid_yaw_.step(error_yaw, dt);  // Go straight, no rotation
                
                // Once 15cm traveled or about to hit a wall, proceed to turn based on decision made above

                //Reset driven distance when going over a black line = intersection edge
                if (line_detection_ > 0) {
                    last_encoders_ = encoders_;
                    //RCLCPP_INFO(get_logger(),"Line detected - Distance Reset");
                }

                if (lidar_vals_.front < free_space){
                    if (lidar_vals_.front <= wall) {
                        cmd_vel_ = {0.0, 0};
                        state_ = corridor_state::INTERSECTION;
                        send_reset_yaw();
                        last_encoders_ = encoders_;
                        RCLCPP_INFO(get_logger(),"Intersection entered");
                    }
                } 
                else if (std::abs(act_coords_.x) >= 20e-2) { 

                        cmd_vel_ = {0.0, 0};
                        state_ = corridor_state::INTERSECTION;
                        last_encoders_ = encoders_;
                        send_reset_yaw();
                        RCLCPP_INFO(get_logger(),"Intersection entered");
                }
                break;
                

            case corridor_state::EXIT_INTERSECTION:
                cmd_vel_.v = forward_speed_corridor;   
                cmd_vel_.w = pid_yaw_.step(error_yaw, dt);

                //RCLCPP_INFO(get_logger(), "Distance driven: %lf", std::abs(act_coords_.x));

                // Condition: walls detected or moved 25 cm
                if ((lidar_vals_.left < free_space && lidar_vals_.right < free_space) || 
                    std::abs(act_coords_.x) >= 30e-2) {

                    state_ = corridor_state::CORRIDOR_FOLLOWING;
                
                }

                break;

            case corridor_state::TURNING:
                // Use IMU to track rotation
                // Rotate until yaw changes by ±90°
                // Then return to CORRIDOR_FOLLOWING

                //RCLCPP_INFO(get_logger(), "Error yaw: %lf",  error_yaw);Y
                cmd_vel_.w = pid_yaw_turn_.step(error_yaw, dt);
                

                if (abs(error_yaw) <= 0.01) {

                    cmd_vel_ = {0.0, 0};
                    pid_yaw_.reset();
                    set_yaw_ = 0;
                    send_reset_yaw();
                    //last_coords_ = coords_;
                    last_encoders_ = encoders_;
                    state_ = corridor_state::EXIT_INTERSECTION;
                    RCLCPP_INFO(get_logger(),"Turn complete, exiting");
                }

                break;

            case corridor_state::RESET:

                cmd_vel_ = {0, 0};
                set_yaw_ = 0;
                send_reset_yaw();
                last_encoders_ = encoders_;
                pid_yaw_.reset();
                pid_centering_.reset();
                exit_ = algorithms::ArucoID::NONE;
                treasure_ = algorithms::ArucoID::NONE;
                //exiting_corridor
                state_ = corridor_state::WAIT;

                break;

        }

    }

    bool CorridorNav::handle_direction(algorithms::ArucoID& code) {
    switch (code) {
        case algorithms::ArucoID::NONE:
            return false;

        case algorithms::ArucoID::RIGHT:
            if (lidar_vals_.right > intersection_threshold) {
                set_yaw_ = yaw_estimate_ - M_PI/2;
                state_ = corridor_state::TURNING;
                exit_ = algorithms::ArucoID::NONE;
                treasure_ = algorithms::ArucoID::NONE;
                RCLCPP_INFO(get_logger(), "Going right based on tag");
                return true;
            }
            break;

        case algorithms::ArucoID::LEFT:
            if (lidar_vals_.left > intersection_threshold) {
                set_yaw_ = yaw_estimate_ + M_PI/2;
                state_ = corridor_state::TURNING; 
                exit_ = algorithms::ArucoID::NONE;
                treasure_ = algorithms::ArucoID::NONE;
                RCLCPP_INFO(get_logger(), "Going left based on tag");
                return true;
            }
            break;

        case algorithms::ArucoID::STRAIGHT:
            if (lidar_vals_.front > wall &&
                (lidar_vals_.left > intersection_threshold ||
                 lidar_vals_.right > intersection_threshold)) {

                set_yaw_ = yaw_estimate_;
                state_ = corridor_state::CORRIDOR_FOLLOWING;
                exit_ = algorithms::ArucoID::NONE;
                treasure_ = algorithms::ArucoID::NONE;
                RCLCPP_INFO(get_logger(), "Continuing forward based on tag");
                return true;
            }
            break;
    }

    return false;
}
}