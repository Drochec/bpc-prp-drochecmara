#include "imu_node.hpp"

namespace nodes {
    constexpr double imu_dt = 20e-3;
    void ImuNode::on_imu_msg(const sensor_msgs::msg::Imu::SharedPtr msg) {

        float gyro_z = msg->angular_velocity.z;

        if (mode_ == ImuNodeMode::CALIBRATE) {
            gyro_calibration_samples_.push_back(gyro_z);
        }
        //TODO: Calculate dt from header stamp
        //auto dt = msg->header.stamp.nanosec;

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
}
