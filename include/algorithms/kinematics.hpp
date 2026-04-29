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
        uint32_t l; //left
        uint32_t r; //right
    };
    struct Coordinates{ //Cartesian coordinates
        float x;
        float y;
        float fi;
    };

    //TODO: Zmenit chovani Kinematics, tak aby vsechny tridy byli static
    //      Prenechat tak hodnoty tak ukladani hodnot prislusnym nodam
    class Kinematics{
        double wheel_radius_; //m
        double wheel_base_; //m
        int TPR_; //TPR

        Coordinates pose_; //x,y
        Encoders encoders_; //ticks_r,ticks_l
        WheelSpeed wheel_speed_; //w_l [rad/s], w_r [rad/s]
        RobotSpeed robot_speed_; //v [m/s], w [rad/s]
        
    public:
        Kinematics(double wheel_radius, double wheel_base, int ticks_revolution) : wheel_radius_(wheel_radius), wheel_base_(wheel_base), TPR_(ticks_revolution),
        pose_{0.0f, 0.0f, 0.0f},
        encoders_{0, 0},
        wheel_speed_{0.0f, 0.0f},
        robot_speed_{0.0f, 0.0f} {};
        RobotSpeed forward(WheelSpeed x) const;
        WheelSpeed inverse(RobotSpeed x) const;
        Coordinates forward(Encoders x) const;
        Encoders inverse(Coordinates x) const;
        float calculate_distance_traveled(const Encoders& start_encoders, const Encoders& current_encoders) const;
    };
}