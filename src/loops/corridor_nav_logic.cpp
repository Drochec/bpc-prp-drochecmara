#include "loops/corridor_nav.hpp"

namespace loops {

    bool CorridorNav::handle_direction(algorithms::ArucoID& code) {
    switch (code) {
        case algorithms::ArucoID::NONE:
            return false;

        case algorithms::ArucoID::RIGHT:
            if (lidar_vals_.right > free_space) {
                set_yaw_ = yaw_estimate_ - M_PI/2;
                state_ = corridor_state::TURNING;
                code = algorithms::ArucoID::NONE;
                kinematics_.reset_pose(encoders_);
                pose_ = {0,0,0};
                turn_start_time_ = now();
                RCLCPP_INFO(get_logger(), "Going right based on tag");
                return true;
            }
            break;

        case algorithms::ArucoID::LEFT:
            if (lidar_vals_.left > free_space) {
                set_yaw_ = yaw_estimate_ + M_PI/2;
                state_ = corridor_state::TURNING; 
                code = algorithms::ArucoID::NONE;
                kinematics_.reset_pose(encoders_);
                pose_ = {0,0,0};
                turn_start_time_ = now();
                RCLCPP_INFO(get_logger(), "Going left based on tag");
                return true;
            }
            break;

        case algorithms::ArucoID::STRAIGHT:
            if (lidar_vals_.front > wall &&
                (lidar_vals_.left > free_space ||
                 lidar_vals_.right > free_space)) {

                //set_yaw_ = yaw_estimate_;
                state_ = corridor_state::CORRIDOR_FOLLOWING;
                code = algorithms::ArucoID::NONE;
                kinematics_.reset_pose(encoders_);
                pose_ = {0,0,0};
                RCLCPP_INFO(get_logger(), "Continuing forward based on tag");
                return true;
            }
            break;
    }

    return false;
}

    std::array<bool, 3> CorridorNav::check_valid_paths(){

        // Check intersections for availible paths
        RCLCPP_INFO(get_logger(),"Checking for valid paths");
        RCLCPP_INFO(get_logger(),"lidar: %f, %f, %f, %f",lidar_vals_.front,lidar_vals_.back,lidar_vals_.left,lidar_vals_.right);
        RCLCPP_INFO(get_logger(),"intersection: %f, %f",intersection_vals_.left,intersection_vals_.right);

        std::array<bool, 3> valid_paths = {false,false,false};

        if (lidar_vals_.front > free_space) valid_paths[0] = true;
        if (lidar_vals_.left > free_space) valid_paths[1] = true;
        if (lidar_vals_.right > free_space) valid_paths[2] = true;

        auto msg = std_msgs::msg::UInt8MultiArray();
        msg.data.reserve(3);

        for (auto const& path : valid_paths) {
            msg.data.push_back(path ? 1 : 0);
        }

        publisher_valid_paths_->publish(msg);

        return valid_paths;
    }

    float CorridorNav::yaw_from_valid_path(const std::array<bool, 3> valid_paths) {
        // Turn right
        if(valid_paths[2]) {
            RCLCPP_INFO(get_logger(),"Going right");
            return -M_PI/2;
        }

        if(valid_paths[0]) {
            RCLCPP_INFO(get_logger(), "Continuing straight");
            return 0;
        }
     
        //Turn left
        if (valid_paths[1]){
            RCLCPP_INFO(get_logger(),"Going left");
            return M_PI/2; 
        }

        //Dead end
        else {
            RCLCPP_INFO(get_logger(),"Dead-end, turning back");
            return M_PI;
        }
    }

}
