#include "pid.hpp"

namespace algorithms {

    Pid::Pid(float kp, float ki, float kd, float th) : 
        kp_(kp),
        ki_(ki),
        kd_(kd), 
        th_(th), 
        prev_error_(0), 
        integral_(0), 
        prev_derivative_(0) {}


    Pid::Pid(float kp, float ki, float kd) : 
        kp_(kp), 
        ki_(ki), 
        kd_(kd), 
        th_(3*10e-3), 
        prev_error_(0), 
        integral_(0), 
        prev_derivative_(0) {}
    
    void Pid::reset() {
            prev_error_ = 0;
            integral_ = 0;
            prev_derivative_ = 0;
        }

    float Pid::step(double error, float dt) {
            if (std::isnan(error)) {
                error = prev_error_;
            }

            integral_ += error * dt;

            //double derivative = (error - prev_error_) / dt;
            //Using filtered derivative
            float derivative = th_ / (th_ + dt) * prev_derivative_ + kd_ / (th_ + dt) * (error - prev_error_); 

            float output = kp_ * error + ki_ * integral_ + kd_ * derivative;

            prev_error_ = error;
            return output;
        }
    }