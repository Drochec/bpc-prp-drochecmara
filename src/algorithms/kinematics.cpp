#include "kinematics.hpp"

#include <cmath>

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
        // compute signed delta with wraparound
        int32_t delta_l = int32_t(encoder_new.l - encoders_.l);
        if (delta_l > 2147483647) delta_l -= 4294967296;  // wrap backward
        else if (delta_l < -2147483648) delta_l += 4294967296; // wrap forward

        int32_t delta_r = int32_t(encoder_new.r - encoders_.r);
        if (delta_r > 2147483647) delta_r -= 4294967296;
        else if (delta_r < -2147483648) delta_r += 4294967296;

        // convert ticks to distance
        float d_L = float(delta_l) * M_PI * wheel_radius_ / TPR_;
        float d_R = float(delta_r) * M_PI * wheel_radius_ / TPR_;

        // check for straight motion
        if (fabs(d_R - d_L) < 1e-6f) {
            float d = (d_R + d_L) / 2.0f;
            Coordinates new_pose;
            new_pose.x = pose_.x + d * cos(pose_.fi);
            new_pose.y = pose_.y + d * sin(pose_.fi);
            new_pose.fi = pose_.fi;
            return new_pose;
        }
        else {
            float d = (d_R + d_L) / 2.0f;
            float d_fi = (d_R - d_L) / wheel_base_;
            float fi_mid = pose_.fi + d_fi / 2.0f;

            return Coordinates{
                static_cast<float>(pose_.x + d * cos(fi_mid)),
                static_cast<float>(pose_.y + d * sin(fi_mid)),
                static_cast<float>(pose_.fi + d_fi)
            };
        }
    }

    Encoders Kinematics::inverse(Coordinates new_pos) const {
        float dx = new_pos.x - pose_.x;
        float dy = new_pos.y - pose_.y;
        float d_fi = new_pos.fi - pose_.fi;

        if (fabs(d_fi) < 1e-6f) {
            // straight motion
            float d = hypot(dx, dy);
            int32_t delta = std::lround(TPR_ * d / (M_PI * wheel_radius_));
            uint32_t new_l = encoders_.l + delta;
            uint32_t new_r = encoders_.r + delta;
            return Encoders{new_l, new_r};
        }
        else {
            // turning motion
            float fi_mid = pose_.fi + d_fi / 2.0f;
            float d = dx * cos(fi_mid) + dy * sin(fi_mid);

            float d_R = d + (wheel_base_ / 2.0f) * d_fi;
            float d_L = d - (wheel_base_ / 2.0f) * d_fi;

            int32_t delta_l = std::lround(TPR_ * d_L / (M_PI * wheel_radius_));
            int32_t delta_r = std::lround(TPR_ * d_R / (M_PI * wheel_radius_));

            uint32_t new_l = encoders_.l + delta_l;
            uint32_t new_r = encoders_.r + delta_r;

            return Encoders{new_l, new_r};
        }
    }
}
