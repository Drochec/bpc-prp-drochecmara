#pragma once
#include <cstdint>

namespace algorithms {

    float inline constexpr max_speed = 19.5f; //rad/s
    double inline constexpr wheel_radius = 68.55e-3; //m
    double inline constexpr wheel_base = 130.00e-3; //m
    int inline constexpr TPR = 585; //Ticks per revolution

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
        Kinematics(double wheel_radius, double wheel_base, int ticks_revolution);
        RobotSpeed forward(WheelSpeed x) const;
        WheelSpeed inverse(RobotSpeed x) const;
        Coordinates forward(Encoders x) const;
        Encoders inverse(Coordinates x) const;
        Coordinates absolute_forward(Encoders last_encoders, Encoders new_encoders) const;
        void reset_pose(Encoders encoders); //Resets accumulated pose, encoders needed to set new base encoder value
    };
}