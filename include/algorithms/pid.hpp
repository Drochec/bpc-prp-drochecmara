#pragma once

#include <iostream>
#include <chrono>

namespace algorithms {

    class Pid {
    public:
        Pid(float kp, float ki, float kd, float thau)
            : kp_(kp), ki_(ki), kd_(kd), thau_(thau), prev_error_(0), integral_(0), derivative_prev_(0) {}

        float step(float error, float dt) {
            if (std::isnan(error)) {
                error = prev_error_;
            }
            integral_ += error * dt;

            //float derivative = (error - prev_error_) / dt;
            float derivative = thau_ / (thau_ + dt) * derivative_prev_ + kd_ / (thau_ + dt) *(error - prev_error_);

            float output = kp_ * error + ki_ * integral_ + kd_ * derivative;
            prev_error_ = error;
            return output;
        }

        void reset() {
            prev_error_ = 0;
            integral_ = 0;
            derivative_prev_ = 0;
        }

    private:
        float kp_;
        float ki_;
        float kd_;
        float prev_error_;
        float integral_;
        float derivative_prev_;
        float thau_;
    };
}
