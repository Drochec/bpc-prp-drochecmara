#include "imu_node.hpp"

namespace nodes {

    constexpr double imu_dt = 20e-3;


    ImuNode::ImuNode() : rclcpp::Node("imu_node"), mode_(ImuNodeMode::CALIBRATE), last_sec_(0), last_nanosec_(0) {
        imu_subscriber_ = create_subscription<sensor_msgs::msg::Imu>(
            Topic::imu,
            10,
            std::bind(&ImuNode::on_imu_msg, this, std::placeholders::_1));
        publisher_ = create_publisher<std_msgs::msg::Float32>(Topic::yaw_estimate,rclcpp::SensorDataQoS());
        timer_ = create_wall_timer(20ms, std::bind(&ImuNode::publish_estimate, this));
        calib_timer_ = create_wall_timer(5s,std::bind(&ImuNode::calibrate, this));
        calib_timer_->cancel();
        reset_yaw_service_ = create_service<prp_project::srv::ResetYawTrigger>(
            "reset_yaw",
            std::bind(&ImuNode::reset_yaw_handle, this, std::placeholders::_1, std::placeholders::_2)
        );
        
        calibrate_service_ = create_service<prp_project::srv::CalibrateTrigger>(
            "calibrate",
            std::bind(&ImuNode::calibrate_handle, this, std::placeholders::_1, std::placeholders::_2)
        );
            
    }

    void ImuNode::on_imu_msg(const sensor_msgs::msg::Imu::SharedPtr msg) {

        float gyro_z = msg->angular_velocity.z;

        if (mode_ == ImuNodeMode::CALIBRATE) {
            gyro_calibration_samples_.push_back(gyro_z);
        }

        else {
           
            planar_integrator_.update(gyro_z,imu_dt);

        }

    }
    void ImuNode::calibrate() {
        planar_integrator_.setCalibration(gyro_calibration_samples_);
        mode_=ImuNodeMode::INTEGRATE;
        calib_timer_->cancel();
    }

    void ImuNode::integrate() {

    }
    // Set the IMU mode
    void ImuNode::setMode(ImuNodeMode mode) {
        mode_=mode;
    }

    // Get the current IMU mode
    ImuNodeMode ImuNode::getMode() {
        return mode_;
    }

    // Get the results after integration
    auto ImuNode::getIntegratedResults() {
        return planar_integrator_.getYaw();
    }

    // Reset the class
    void ImuNode::reset_imu() {
        mode_=ImuNodeMode::CALIBRATE;
        planar_integrator_.reset();
        calib_timer_->reset();
    }

    void ImuNode::publish_estimate() {

        auto msg = std_msgs::msg::Float32();

        if (mode_ == ImuNodeMode::INTEGRATE) {

            msg.data = getIntegratedResults();
        }
        else {
            msg.data = std::numeric_limits<double>::quiet_NaN();
        }

        publisher_->publish(msg);
    }


    //Services

    void ImuNode::calibrate_handle(
        const std::shared_ptr<prp_project::srv::CalibrateTrigger::Request> request,
        std::shared_ptr<prp_project::srv::CalibrateTrigger::Response> response) {

        
        (void)request;  // if unused

        RCLCPP_INFO(this->get_logger(), "Calibration requested");

        // Do your calibration logic here

        reset_imu();
        //Make sure to publish NaN
        publish_estimate();

        response->success = true;
        response->message = "Calibration started";
    }

    void ImuNode::reset_yaw_handle(
        const std::shared_ptr<prp_project::srv::ResetYawTrigger::Request> request,
        std::shared_ptr<prp_project::srv::ResetYawTrigger::Response> response) {

            (void)request;

            RCLCPP_INFO(get_logger(), "Yaw reset requested");

            planar_integrator_.reset();

            response->success = true;
            response->message = "Yaw reset";
        }
}

