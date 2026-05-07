#pragma once

#include <iostream>
#include <rclcpp/rclcpp.hpp>

namespace algorithms {

    class Pid {
    public:
        Pid(float kp, float ki, float kd, float th);

        Pid(float kp, float ki, float kd);

        float step(double error, float dt);

        void reset();
    private:
        float kp_;
        float ki_;
        float kd_;
        float th_;

        double prev_error_;
        double integral_;
        double prev_derivative_;
        
    };
}
