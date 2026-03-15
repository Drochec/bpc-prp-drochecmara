#include <gamepad_node.hpp>

namespace nodes{

    void JoyNode::publish_speed(){
    
    //Nastaveni promennych
    auto msg = std_msgs::msg::Float32MultiArray(); 

    SDL_PollEvent(&joy_event_);

    //Kontrola joy eventu, a ulozeni standardizovane hodnoty
    if(joy_event_.type == SDL_JOYAXISMOTION) {
        if (joy_event_.jaxis.axis == left_stick_axis){
            int joy_left_stick_raw = -joy_event_.jaxis.value;

            left_stick_val_ = standardize(joy_left_stick_raw,-32768,32767,-max_v,max_v);            
        }
        
        
        if (joy_event_.jaxis.axis == right_stick_axis){
            int joy_right_stick_raw = -joy_event_.jaxis.value;

            right_stick_val_ = standardize(joy_right_stick_raw,-32768,32767,-max_w,max_w);
        }

    }

    //Publikovani

    msg.data = {left_stick_val_,right_stick_val_};

    publisher_->publish(msg);
}

}