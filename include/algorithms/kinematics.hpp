#pragma once
#include <cstdint>

namespace algorithms {
    struct RobotSpeed{
        float v; //linear
        float w; //angluar
    };

    struct WheelSpeed{ //depends on you in what units
        float l; //left
        float r; //right
    };

    struct Encoders{
        uint16_t l; //left
        uint16_t r; //right
    };
    struct Coordinates{ //Cartesian coordinates
        float x;
        float y;
        float fi;
    };

    class Kinematics{
        double wheel_radius_; //mm
        double wheel_base_; //mm
        int TPR_; //TPR

        Coordinates pose_; //x,y
        Encoders encoders_; //ticks_r,ticks_l

        Kinematics(double wheel_radius, double wheel_base, int ticks_revolution) : wheel_radius_(wheel_radius), wheel_base_(wheel_base), TPR_(ticks_revolution) {} ;
        RobotSpeed forward(WheelSpeed x) const;
        WheelSpeed inverse(RobotSpeed x) const;
        Coordinates forward(Encoders x) const;
        Encoders inverse(Coordinates x) const;
    };
}