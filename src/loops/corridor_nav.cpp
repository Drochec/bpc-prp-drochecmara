#include "corridor_nav.hpp"
#include <math.h>
#include <sys/stat.h>

namespace loops {

    CorridorNav::CorridorNav() : rclcpp::Node("corridor_nav"),
                        cmd_vel_({0,0}),

                        lidar_vals_({0,0,0,0}),
                        intersection_vals_({0,0,0,0}),
                        yaw_estimate_(0),
                        line_detection_(0),
                        pose_({0,0,0}),
                        encoders_({0, 0}),
                        last_encoders_({0,0}),

                        state_(corridor_state::WAIT),
                        last_state_(corridor_state::RESET),
                        set_yaw_(0),
                        exit_(algorithms::ArucoID::NONE),
                        treasure_(algorithms::ArucoID::NONE),
                        heading_to_exit_(false),
                        last_time_(0),
                        first_run_(true),
                        valid_paths_({false,false,false,false}),

                        kinematics_(algorithms::wheel_radius,algorithms::wheel_base,algorithms::TPR),
                        pid_yaw_(4,0,0,10*loops::dt),
                        pid_yaw_turn_(6,0.5,0,3*loops::dt),
                        pid_centering_(3,0,0,3*loops::dt) //thau = 3*dt
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


    void CorridorNav::send_calibrate_trigger(){

        auto request = std::make_shared<prp_project::srv::CalibrateTrigger::Request>();

        calibrate_client_->async_send_request(request);
    }
    
    void CorridorNav::send_reset_yaw(){

        auto request = std::make_shared<prp_project::srv::ResetYawTrigger::Request>();

        reset_yaw_client_->async_send_request(request);
        kinematics_.reset_pose(encoders_);
        pose_ = kinematics_.forward(encoders_);
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

        //RCLCPP_INFO(get_logger(),"Yaw - IMU: %lf Enc: %lf, CF: %lf",yaw_deg,pose_deg,yaw_estimate_filtered_);
    }

    
    void CorridorNav::encoders_callback(std_msgs::msg::UInt32MultiArray::SharedPtr msg) {
        encoders_ = {msg->data[0], msg->data[1]};

        pose_ = kinematics_.forward(encoders_);
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
