#include <corridor_nav.hpp>
#include <math.h>
#include <sys/stat.h>

#include "imu_node.hpp"
#include "lidar_node.hpp"

// Constants for encoder distance calculation
constexpr double WHEEL_RADIUS = 68.55e-3;  // meters
constexpr int TPR = 585;  // Ticks per revolution

constexpr float dt = 20e-3; //20ms between calls

constexpr float forward_speed_corridor = 0.075;
constexpr float wall_threshold = 0.55;
constexpr float front_stop = 0.25;
constexpr float exit_centering_error = 0.01;
constexpr float intersection_advance_distance = 0.15;  // 15cm in meters

namespace loops {


    void CorridorNav::state_machine() {
        
        if (state_ != last_state_) {
            RCLCPP_INFO(get_logger(), "State: %u",  static_cast<unsigned int>(state_));
            last_state_ = state_;
        }

        auto error_lidar = lidar_vals_.left - lidar_vals_.right;
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


                //Intersection check - Wall infront or Line detected
                /*
                if ((lidar_vals_.front < front_stop) || (line_detection_ > 0)) {
                        cmd_vel_ = {0, 0};
                        state_ = corridor_state::INTERSECTION;
                        break;
                    }
                        */
                /*
                if (lidar_vals_.left > wall_threshold || lidar_vals_.right > wall_threshold) {

                    if (lidar_vals_.front < wall_threshold) {
                        if (lidar_vals_.front <= front_stop) {
                            cmd_vel_ = {0, 0};
                            state_ = corridor_state::INTERSECTION;
                            break;
                        }
                    }

                    cmd_vel_ = {0, 0};
                    state_ = corridor_state::INTERSECTION;
                    break;
                }*/

                /*
                else if (lidar_vals_.left < 0.15 || lidar_vals_.right < 0.15) {
                    state_ = corridor_state::CENTERING;
                    RCLCPP_INFO(get_logger(),"Centering using lidar");
                    break;
                }
                    */
                
                //Corridor following
                cmd_vel_.v = forward_speed_corridor;
                //cmd_vel_.w = pid_yaw_.step(error_yaw, dt);
                cmd_vel_.w = pid_centering_.step(error_lidar, dt);

                break;

            case corridor_state::CENTERING:
                // Safety: if we lose walls → bail out
                /*
                if (!(lidar_vals_.left < wall_threshold &&
                      lidar_vals_.right < wall_threshold)) {
                    state_ = corridor_state::CORRIDOR_FOLLOWING;
                    break;
                    }
                */
                // Aggressive controller
                cmd_vel_.w = pid_centering_.step(error_lidar, dt);
                RCLCPP_INFO(get_logger(), "Error center: %lf",  error_lidar);

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

                // Check intersections for availible paths
                RCLCPP_INFO(get_logger(),"Checking for intersections");
                RCLCPP_INFO(get_logger(),"lidar: %f, %f, %f, %f",lidar_vals_.front,lidar_vals_.back,lidar_vals_.left,lidar_vals_.right);
                RCLCPP_INFO(get_logger(),"intersection: %f, %f",intersection_vals_.left,intersection_vals_.right);

                //If left or right, need to enter the intersection 
                // Turn right
                if(intersection_vals_.right > wall_threshold) {
                    set_yaw_ = yaw_estimate_ - M_PI/2;
                    RCLCPP_INFO(get_logger(),"Going right");
                }
                //turn left
                else if (intersection_vals_.left > wall_threshold) {
                    set_yaw_ = yaw_estimate_ + M_PI/2;
                    RCLCPP_INFO(get_logger(),"Going left");
                }

                // Not inside a corridor, go back to following 
                else if (lidar_vals_.front > front_stop) {
                    state_ = corridor_state::CORRIDOR_FOLLOWING;
                    RCLCPP_INFO(get_logger(),"None detected");
                    break;
                }

                //turn back
                else {
                    state_ = corridor_state::TURNING;
                    set_yaw_ = yaw_estimate_ + M_PI;
                    pid_yaw_turn_.reset();
                    RCLCPP_INFO(get_logger(),"Dead-end, turning back");
                    break;
                }

                // Save encoder position and reset distance tracker
                encoders_at_intersection_start_ = encoders_;
                distance_traveled_at_intersection_ = 0.0f;
                
                // Transition to advance state to move 15cm before committing to turn
                //pid_yaw_.reset();
                state_ = corridor_state::INTERSECTION_ADVANCE;
                last_coords_ = coords_;
                RCLCPP_INFO(get_logger(),"Entering intersection");
                

            case corridor_state::INTERSECTION_ADVANCE:
                // Move forward while tracking distance via encoders
                cmd_vel_.v = forward_speed_corridor;
                cmd_vel_.w = 0.0;  // Go straight, no rotation
                
                /* 
                // Calculate distance traveled since intersection detection
                distance_traveled_at_intersection_ = calculate_distance_from_encoders(
                    encoders_at_intersection_start_,
                    encoders_
                );
                */
                // Once 15cm traveled or about to hit a wall, proceed to turn based on decision made above
                if (std::abs(coords_.x - last_coords_.x) >= 15e-2 || lidar_vals_.front < front_stop) {
                    cmd_vel_ = {0.0, 0};
                    pid_yaw_turn_.reset();
                    state_ = corridor_state::TURNING;
                    
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

                //RCLCPP_INFO(get_logger(), "Error yaw: %lf",  error_yaw);Y
                cmd_vel_.w = pid_yaw_turn_.step(error_yaw, dt);
                

                if (abs(error_yaw) <= 0.01) {

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
    CorridorNav::CorridorNav() : rclcpp::Node("corridor_nav"),
                        cmd_vel_({0,0}),
                        lidar_vals_({0,0,0,0}),
                        intersection_vals_({0,0,0,0}),
                        yaw_estimate_(0),
                        set_yaw_(0),
                        exiting_corridor_(false),
                        encoders_({0, 0}),
                        encoders_at_intersection_start_({0, 0}),
                        distance_traveled_at_intersection_(0),
                        line_detection_(0),
                        coords_({0,0,0}),
                        last_coords_({0,0,0}),
                        next_turn_direction_state_(corridor_state::TURNING),
                        state_(corridor_state::WAIT),
                        last_state_(corridor_state::RESET),
                        pid_yaw_(3,0.1,0.1),
                        pid_yaw_turn_(2.5,0.2,0),
                        pid_centering_(15,0.5,0.8,3*20*1e-3) //thau = 3*dt
        {

            subscriber_range_est_ = create_subscription<std_msgs::msg::Float32MultiArray>(
                Topic::range_estimate,
                15,
                std::bind(&CorridorNav::range_est_callback,this, std::placeholders::_1)
            );

            subscriber_intersection_range_ = create_subscription<std_msgs::msg::Float32MultiArray>(
                Topic::intersect_estimate,
                15,
                std::bind(&CorridorNav::intersection_range_callback,this, std::placeholders::_1)
            );

            subscriber_yaw_est_ = create_subscription<std_msgs::msg::Float32>(
                Topic::yaw_estimate,
                15,
                std::bind(&CorridorNav::yaw_est_callback,this, std::placeholders::_1)
            );

            /*
            subscriber_encoders_ = create_subscription<std_msgs::msg::UInt32MultiArray>(
                Topic::encoders,
                15,
                std::bind(&CorridorNav::encoder_callback,this, std::placeholders::_1)
            );

            subscriber_line_ = create_subscription<std_msgs::msg::UInt8>(
                Topic::line_estimate_discrete,
                5,
                std::bind(&CorridorNav::line_callback, this, std::placeholders::_1)
            );
            */
            subscriber_coords_ = create_subscription<std_msgs::msg::Float32MultiArray>(
                Topic::coords,
                5,
                std::bind(&CorridorNav::coords_callback, this, std::placeholders::_1)
            );



            publisher_cmd_vel_ = create_publisher<std_msgs::msg::Float32MultiArray>(Topic::cmd_vel,5);

            publish_timer_ = create_wall_timer(15ms, std::bind(&CorridorNav::publish_cmd_vel,this));

            decision_timer_ = create_wall_timer(20ms, std::bind(&CorridorNav::state_machine,this));

            calibrate_client_ = create_client<prp_project::srv::CalibrateTrigger>("calibrate");
            reset_yaw_client_ = create_client<prp_project::srv::ResetYawTrigger>("reset_yaw");

            button_cmd_service_ = create_service<prp_project::srv::ButtonCmd>(
                "button_cmd",
                std::bind(&CorridorNav::button_cmd_handle,this,std::placeholders::_1,std::placeholders::_2)
            );

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
    void CorridorNav::intersection_range_callback(std_msgs::msg::Float32MultiArray::SharedPtr msg) {

        intersection_vals_.left = msg->data[0];
        intersection_vals_.right = msg->data[1];

    }

    void CorridorNav::yaw_est_callback(std_msgs::msg::Float32::SharedPtr msg) {

        yaw_estimate_ = msg->data;
    }

    /*
    void CorridorNav::encoder_callback(std_msgs::msg::UInt32MultiArray::SharedPtr msg) {
        encoders_.l = msg->data[0];
        encoders_.r = msg->data[1];
    }

    void CorridorNav::line_callback(std_msgs::msg::UInt8::SharedPtr msg) {
        line_detection_ = msg->data;
    }
*/
    void CorridorNav::coords_callback(std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        coords_ = {msg->data[0],msg->data[1],msg->data[2]};
    }

}
