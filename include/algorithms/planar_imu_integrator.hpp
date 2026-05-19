#pragma once

#include <iostream>
#include <cmath>
#include <numeric>
#include <chrono>

namespace algorithms {

    
    class PlanarImuIntegrator {
    public:

        PlanarImuIntegrator(float alpha) : yaw_filtered_(0.0f), gyro_offset_(0.0f), alpha_(alpha) {}
        PlanarImuIntegrator() : yaw_filtered_(0.0f), gyro_offset_(0.0f), alpha_(0.90) {}

        //Integrate using complemtary filtering
        void update(float gyro_z, float correction_yaw, double dt) {
            float yaw = yaw_filtered_ + (gyro_z * dt);

            yaw_filtered_ = yaw * alpha_ + (1.0F - alpha_) * correction_yaw;

        }

        // TODO: Calibrate the gyroscope by computing average from static samples
        void setCalibration(std::vector<float> gyro) {
            gyro_offset_ = std::accumulate(gyro.begin(), gyro.end(), 0.0f) / gyro.size();
        }

        // TODO: Return the current estimated yaw
        [[nodiscard]] float getYaw() const {
            return yaw_filtered_;
        }

        // TODO: Reset orientation and calibration
        void reset() {
            yaw_filtered_ = 0.0f;
            gyro_offset_ = 0.0f;
        }

    private:
        float yaw_filtered_;       // Integrated yaw angle (radians)
        float gyro_offset_; // Estimated gyro bias
        float alpha_;       // Complementary filter constant, higher means trusts integrated value more
    };
}

