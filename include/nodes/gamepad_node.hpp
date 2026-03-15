#pragma once

#include <helper.hpp>
#include <string>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <SDL2/SDL.h>

//Kod prevzaty a upraveny z:
//https://gist.github.com/fabiocolacio/6af2ef76a706443adb210d23bd036d04

using namespace std::chrono_literals;

namespace nodes {
    //Zmenit podle gamepadu
    int constexpr left_stick_axis = 1;
    int constexpr right_stick_axis = 3;
    float constexpr max_v = 2;
    float constexpr max_w = 19;

    class JoyNode : public rclcpp::Node {
   
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;  
    SDL_Event joy_event_;

    float left_stick_val_;
    float right_stick_val_;
    
    public:
        JoyNode() : rclcpp::Node("joy_node"), left_stick_val_(0), right_stick_val_(0) {
            
            //Inicializace nody
            publisher_ = create_publisher<std_msgs::msg::Float32MultiArray>(Topic::cmd_vel, 1);
            timer_ = create_wall_timer(10ms, std::bind(&JoyNode::publish_speed, this));
            
            
            //Inicializace gamepadu
            int joysticks = SDL_Init(SDL_INIT_JOYSTICK);
            
            //Prezaty kod
            // If there was an error setting up the joystick subsystem, quit.
            if (joysticks < 0) {
                RCLCPP_INFO(get_logger(),"Unable to initialize the joystick subsystem.\n");
            }

            // Check how many joysticks are connected.
            joysticks = SDL_NumJoysticks();
            RCLCPP_INFO(get_logger(),"There are %d joysticks connected.\n", joysticks);

            // If there are joysticks connected, open one up for reading
            if (joysticks > 0) {
                if (SDL_JoystickOpen(0) == NULL) {
                    RCLCPP_INFO(get_logger(),"There was an error reading from the joystick.\n");
                }
            }
            // If there are no joysticks connected, exit the program.
            else {
                RCLCPP_INFO(get_logger(),"There are no joysticks connected.\n");
            }
         }
                //SDL pracuje s pointery, je treba vlastni destruktor
        ~JoyNode() { 
            SDL_Quit();
        }

        void publish_speed();

        static inline float standardize(int x, int x_min,int x_max,float y_min,float y_max) {
            return (y_max - y_min) * (x - x_min)/(x_max-x_min) + y_min;
        }

    };
}