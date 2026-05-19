#include  "line.hpp"
#include <algorithm>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/float32.hpp>

namespace nodes {

        LineNode::LineNode() : rclcpp::Node("line_node"),
        line_estimator_(0.35) {

            publisher_line_discrete_ = create_publisher<std_msgs::msg::UInt8>(Topic::line_estimate_discrete,10);
            //publisher_line_ = create_publisher<std_msgs::msg::Float32>(Topic::line_estimate,10);

            subscriber_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
                    Topic::line,
                    1,
                    std::bind(&LineNode::on_line_sensors_msg, this, std::placeholders::_1)
                );

            timer_ = this->create_wall_timer(20ms,std::bind(&LineNode::publish_line_estimate, this));
            
        };

    void LineNode::on_line_sensors_msg(const std_msgs::msg::UInt16MultiArray::SharedPtr msg) {
        sensor_vals_raw_.left = msg->data[0];
        sensor_vals_raw_.right = msg->data[1];

        float sensor_left_norm = float(sensor_vals_raw_.left - sensor_min_.left) / (sensor_max_.left - sensor_min_.left);
        float sensor_right_norm = float(sensor_vals_raw_.right - sensor_min_.right) / (sensor_max_.right - sensor_min_.right);

        sensor_vals_.left = sensor_left_norm;
        sensor_vals_.right = sensor_right_norm;

        //RCLCPP_INFO(this->get_logger(), "Received, left: %f, right: %f", sensor_vals_.left, sensor_vals_.right);
        //RCLCPP_INFO(this->get_logger(), "Received raw, left: %u, right: %u", sensor_vals_raw_.left, sensor_vals_raw_.right);
        //auto estimated_discrete_pos = algorithms::LineEstimator::estimate_discrete_line_pose(sensor_vals_);
        //auto estimated_line_pos = algorithms::LineEstimator::estimate_continuous_line_pose(sensor_vals_);
        //RCLCPP_INFO(this->get_logger(), "Line pos: %f", estimated_line_pos);
    };

    float LineNode::get_continuous_line_pose() {
        return line_estimator_.estimate_continuous_line_pose(sensor_vals_);
    }

    DiscreteLinePose LineNode::get_discrete_line_pose() {
        return line_estimator_.estimate_discrete_line_pose(sensor_vals_);
        
    }

    void LineNode::publish_line_estimate() {
        auto msg_line_discrete = std_msgs::msg::UInt8();
        msg_line_discrete.data = static_cast<unsigned char>(get_discrete_line_pose());
        
        //auto msg_line = std_msgs::msg::Float32();
        //msg_line.data = get_continuous_line_pose();

        publisher_line_discrete_->publish(msg_line_discrete);
        //publisher_line_->publish(msg_line);
        
    }
}

namespace algorithms {
    DiscreteLinePose LineEstimator::estimate_discrete_line_pose(const SensorNorm& sensor_vals) {

        if (first_run_) {
            smooth_left_ = sensor_vals.left;
            smooth_right_ = sensor_vals.right;
            first_run_ = false;
        }
        else {
            smooth_left_ += alpha_ * (sensor_vals.left - smooth_left_);
            smooth_right_ += alpha_ * (sensor_vals.right - smooth_right_);
        }

        auto line_right = smooth_right_ > treshold;
        auto line_left = smooth_left_ > treshold;

        if (line_right && line_left) 
            return DiscreteLinePose::LineBoth;
        
        else if (line_right)
            return DiscreteLinePose::LineOnRight;
        
        else if (line_left)
            return DiscreteLinePose::LineOnLeft;
        else
            return DiscreteLinePose::LineNone;
        
    }

    float LineEstimator::estimate_continuous_line_pose(const SensorNorm& sensor_vals) {
        float pos = (sensor_vals.left-sensor_vals.right)*sensor_offset; //(-sensor_offset * sensor_vals.left + sensor_offset * sensor_vals.right) / (sensor_vals.left + sensor_vals.right);

        return std::clamp(pos, -1.0f, 1.0f);
    }
}