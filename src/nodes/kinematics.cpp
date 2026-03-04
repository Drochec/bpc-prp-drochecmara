#include "kinematics.hpp"

#include <math.h>

namespace algorithms {
    RobotSpeed Kinematics::forward(WheelSpeed wheel_speed) const {

        float v = (wheel_radius_/2) * (wheel_speed.l + wheel_speed.r);
        float w = (wheel_radius_/wheel_base_) * (wheel_speed.r - wheel_speed.l);

        return RobotSpeed{v, w};

    }
    WheelSpeed Kinematics::inverse(RobotSpeed rbt_speed) const {

        float v_r = (2*rbt_speed.v + rbt_speed.w*wheel_base_) / (2 * wheel_radius_);
        float v_l = (2*rbt_speed.v - rbt_speed.w*wheel_base_) / (2 * wheel_radius_);

        return WheelSpeed{v_l, v_r};
    }

    Coordinates Kinematics::forward(Encoders encoder_new) const {
        float d_L = (encoder_new.l - encoders_.l) / TPR_ * M_PI * wheel_radius_;
        float d_R = (encoder_new.r - encoders_.r) / TPR_ * M_PI * wheel_radius_;

        float d = (d_R + d_L) / 2;
        float d_Fi = (d_R - d_L) / wheel_base_;

        float x_new = pose_.x + d * cos(pose_.fi + d_Fi/2);
        float y_new = pose_.y + d * sin(pose_.fi + d_Fi/2);
        float fi_new = pose_.fi + d_Fi;

        return Coordinates{x_new, y_new, fi_new};
    }
    Encoders Kinematics::inverse(Coordinates new_pos) const {
        if (pose_.fi == new_pos.fi) {
            float d = hypot(new_pos.x-pose_.x, new_pos.y-pose_.y);
            int encoder_update = TPR_/M_PI / wheel_radius_ * d;

            return Encoders{encoders_.r+encoder_update,encoders_.l+encoder_update};
        }

        else {
            float d_x_r = cos(pose_.fi)*(new_pos.x - pose_.x) + sin(pose_.fi)*(new_pos.y - pose_.y);
            float d_fi = new_pos.fi-pose_.fi;
            //float d_y_r = -sin(pose_.fi)*(new_pos.x - pose_.x) + cos(pose_.fi)*(new_pos.y - pose_.y);

            float R = (d_x_r)/(sin(d_fi));

            float d = R * d_fi;

            float d_R = d + wheel_base_/2*d_fi;
            float d_L = d - wheel_base_/2*d_fi;

            int encoder_update_l = TPR_/M_PI / wheel_radius_ * d_L;
            int encoder_update_r = TPR_/M_PI / wheel_radius_ * d_R;

            return Encoders{encoders_.l+encoder_update_l,encoders_.r+encoder_update_r};
        }

    }
}
