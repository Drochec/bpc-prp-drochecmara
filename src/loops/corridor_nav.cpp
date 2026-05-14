#include "corridor_nav.hpp"
#include <math.h>
#include <sys/stat.h>


constexpr float dt = 10e-3; //20ms between calls

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

namespace loops {

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
                

                if (abs(error_yaw) <= 0.025) {

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
    CorridorNav::CorridorNav() : rclcpp::Node("corridor_nav"),
                        cmd_vel_({0,0}),
                        lidar_vals_({0,0,0,0}),
                        intersection_vals_({0,0,0,0}),
                        yaw_estimate_(0),
                        set_yaw_(0),
                        exiting_corridor_(false),
                        turn_set_(false),
                        encoders_({0, 0}),
                        last_encoders_({0,0}),
                        line_detection_(0),
                        act_coords_({0,0,0}),
                        exit_(algorithms::ArucoID::NONE),
                        treasure_(algorithms::ArucoID::NONE),
                        next_turn_direction_state_(corridor_state::TURNING),
                        state_(corridor_state::WAIT),
                        last_state_(corridor_state::RESET),
                        reg_mode_(0),
                        kinematics_(algorithms::wheel_radius,algorithms::wheel_base,algorithms::TPR),
                        pid_yaw_(3,0.1,0.1,10*dt),
                        pid_yaw_turn_(1,1.5,0),
                        pid_centering_(3,0,0,3*dt) //thau = 3*dt
        {

            subscriber_range_est_ = create_subscription<std_msgs::msg::Float32MultiArray>(
                Topic::range_estimate,
                3,
                std::bind(&CorridorNav::range_est_callback,this, std::placeholders::_1)
            );

            subscriber_intersection_range_ = create_subscription<std_msgs::msg::Float32MultiArray>(
                Topic::intersect_estimate,
                3,
                std::bind(&CorridorNav::intersection_range_callback,this, std::placeholders::_1)
            );

            subscriber_yaw_est_ = create_subscription<std_msgs::msg::Float32>(
                Topic::yaw_estimate,
                3,
                std::bind(&CorridorNav::yaw_est_callback,this, std::placeholders::_1)
            );

            
            subscriber_encoders_ = create_subscription<std_msgs::msg::UInt32MultiArray>(
                Topic::encoders,
                1,
                std::bind(&CorridorNav::encoders_callback,this, std::placeholders::_1)
            );
            
            subscriber_line_ = create_subscription<std_msgs::msg::UInt8>(
                Topic::line_estimate_discrete,
                5,
                std::bind(&CorridorNav::line_callback, this, std::placeholders::_1)
            );

            subscriber_tag_detections_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
                Topic::tag_detections,
                3,
                std::bind(&CorridorNav::tag_detections_callback, this, std::placeholders::_1)
            );
            /*
            subscriber_coords_ = create_subscription<std_msgs::msg::Float32MultiArray>(
                Topic::coords,
                1, 
                std::bind(&CorridorNav::coords_callback, this, std::placeholders::_1)
            );
            */



            publisher_cmd_vel_ = create_publisher<std_msgs::msg::Float32MultiArray>(Topic::cmd_vel,1);

            publish_timer_ = create_wall_timer(5ms, std::bind(&CorridorNav::publish_cmd_vel,this));

            decision_timer_ = create_wall_timer(10ms, std::bind(&CorridorNav::state_machine,this));

            calibrate_client_ = create_client<prp_project::srv::CalibrateTrigger>("calibrate");
            reset_yaw_client_ = create_client<prp_project::srv::ResetYawTrigger>("reset_yaw");

            button_cmd_service_ = create_service<prp_project::srv::ButtonCmd>(
                "button_cmd",
                std::bind(&CorridorNav::button_cmd_handle,this,std::placeholders::_1,std::placeholders::_2)
            );

        }

    bool CorridorNav::handle_direction(algorithms::ArucoID& code) {
    switch (code) {
        case algorithms::ArucoID::NONE:
            return false;

        case algorithms::ArucoID::RIGHT:
            if (lidar_vals_.right > intersection_threshold) {
                set_yaw_ = yaw_estimate_ - M_PI/2;
                state_ = corridor_state::TURNING;
                code = algorithms::ArucoID::NONE;
                RCLCPP_INFO(get_logger(), "Going right based on tag");
                return true;
            }
            break;

        case algorithms::ArucoID::LEFT:
            if (lidar_vals_.left > intersection_threshold) {
                set_yaw_ = yaw_estimate_ + M_PI/2;
                state_ = corridor_state::TURNING;
                code = algorithms::ArucoID::NONE;
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
                code = algorithms::ArucoID::NONE;
                RCLCPP_INFO(get_logger(), "Continuing forward based on tag");
                return true;
            }
            break;
    }

    return false;
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

    
    void CorridorNav::encoders_callback(std_msgs::msg::UInt32MultiArray::SharedPtr msg) {
        encoders_ = {msg->data[0], msg->data[1]};

        act_coords_ = kinematics_.absolute_forward(last_encoders_ , encoders_);
        //RCLCPP_INFO(get_logger(), "Distance driven: %lf", std::abs(act_coords_.x));
    }

    void CorridorNav::line_callback(std_msgs::msg::UInt8::SharedPtr msg) {
        line_detection_ = msg->data;
    }
/*
    void CorridorNav::coords_callback(std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        coords_ = {msg->data[0],msg->data[1],msg->data[2]};
    }
        */

    void CorridorNav::tag_detections_callback(std_msgs::msg::UInt8MultiArray::SharedPtr msg) {

        auto ids = msg->data;
        if (!ids.empty()) {
            for (auto& id_raw : ids) {
                

                if (id_raw < 10) {
                    auto id = static_cast<algorithms::ArucoID>(id_raw);
                    if (exit_ != id){
                        exit_ = id;
                        RCLCPP_WARN(get_logger(),"Got exit: %u",id_raw);
                    }
                }
                else {
                    id_raw %= 10;
                    auto id = static_cast<algorithms::ArucoID>(id_raw);
                    if (treasure_ != id) {
                        treasure_ = id;
                        RCLCPP_WARN(get_logger(),"Got treasure: %u", id_raw);
                    }
                }
            }
        } 

    }
}
