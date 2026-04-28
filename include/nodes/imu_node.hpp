#pragma once

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <sensor_msgs/msg/imu.hpp>
#include "algorithms/planar_imu_integrator.hpp"
#include "helper.hpp"
#include <std_msgs/msg/float32.hpp>
#include "prp_project/srv/calibrate_trigger.hpp"

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
        algorithms::PlanarImuIntegrator planar_integrator_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr calib_timer_;

        std::vector<float> gyro_calibration_samples_;

        void on_imu_msg(const sensor_msgs::msg::Imu::SharedPtr msg);
        void calibrate();
        void integrate();
        void publish_estimate();

    public:
        ImuNode() : rclcpp::Node("imu_node"), mode_(ImuNodeMode::CALIBRATE) {

            imu_subscriber_ = create_subscription<sensor_msgs::msg::Imu>(
                Topic::imu,
                10,
                std::bind(&ImuNode::on_imu_msg, this, std::placeholders::_1));

            publisher_ = create_publisher<std_msgs::msg::Float32>(Topic::yaw_estimate,10);
            timer_ = create_wall_timer(20ms, std::bind(&ImuNode::publish_estimate, this));
            calib_timer_ = create_wall_timer(5s,std::bind(&ImuNode::calibrate, this));

        }
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
