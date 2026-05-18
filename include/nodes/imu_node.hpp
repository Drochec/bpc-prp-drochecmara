#pragma once

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <sensor_msgs/msg/imu.hpp>
#include "algorithms/planar_imu_integrator.hpp"
#include "helper.hpp"
#include <std_msgs/msg/float32.hpp>
#include "prp_project/srv/calibrate_trigger.hpp"
#include "prp_project/srv/reset_yaw_trigger.hpp"

//TODO
//Implement service server

using namespace std::chrono_literals;
namespace nodes {

    enum class ImuNodeMode {
        CALIBRATE,
        INTEGRATE,
    };

    class ImuNode : public rclcpp::Node {
    private:
        ImuNodeMode mode_;

        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
        rclcpp::Service<prp_project::srv::ResetYawTrigger>::SharedPtr reset_yaw_service_;
        rclcpp::Service<prp_project::srv::CalibrateTrigger>::SharedPtr calibrate_service_;

        algorithms::PlanarImuIntegrator planar_integrator_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr calib_timer_;


        std::vector<float> gyro_calibration_samples_;

        void on_imu_msg(const sensor_msgs::msg::Imu::SharedPtr msg);
        void calibrate();
        void integrate();
        void publish_estimate();

        void reset_yaw_handle(const std::shared_ptr<prp_project::srv::ResetYawTrigger::Request> request,
        std::shared_ptr<prp_project::srv::ResetYawTrigger::Response> response);
        void calibrate_handle(
        const std::shared_ptr<prp_project::srv::CalibrateTrigger::Request> request,
        std::shared_ptr<prp_project::srv::CalibrateTrigger::Response> response);

    public:
        ImuNode();
        ~ImuNode() override = default;

        // Set the IMU mode
        void setMode(ImuNodeMode mode);

        // Get the current IMU mode
        ImuNodeMode getMode();

        // Get the results after integration
        auto getIntegratedResults();

        // Reset the class
        void reset_imu();

    };
}
