#include "lidar_node.hpp"

namespace nodes {
    void LidarNode::subscriber_callback(sensor_msgs::msg::LaserScan::SharedPtr msg) {

        auto angle_start = msg->angle_min;
        auto angle_end = msg->angle_max;
        auto points = msg->ranges;
        auto range_max = msg->range_max;
        auto range_min = msg->range_min;

        lidar_filter_results_ = algorithms::LidarFilter::apply_filter(points,angle_start,angle_end,range_max,range_min);

    }

    void LidarNode::publish() {

        auto msg = std_msgs::msg::Float32MultiArray();

        msg.data = {lidar_filter_results_.front, lidar_filter_results_.back, lidar_filter_results_.left, lidar_filter_results_.right};

        publisher_->publish(msg);

    }
}

namespace algorithms {
    LidarFilterResults LidarFilter::apply_filter(std::vector<float> points, float angle_start, float angle_end, float range_max, float range_min) {
        // Create containers for values in different directions
        std::vector<float> left{};
        std::vector<float> right{};
        std::vector<float> front{};
        std::vector<float> back{};

        // TODO: Define how wide each directional sector should be (in radians)
        constexpr float angle_range = M_PI/3;

        // Compute the angular step between each range reading
        auto angle_step = (angle_end - angle_start) / points.size();

        for (size_t i = 0; i < points.size(); ++i) {
            auto angle = angle_start + i * angle_step;

            // TODO: Skip invalid (infinite) readings
            /*if (std::isinf(points.at(i)) || (points.at(i)<(1e-6))){
                points.at(i) = range_max;
            }*/
            if (std::isinf(points.at(i))){
                points.at(i) = range_max;
                //continue;
            }
            if (points.at(i)<(150e-3)){
                //points.at(i) = range_min;
                continue;
            }

            // TODO: Sort the value into the correct directional bin based on angle
            if (-M_PI/6 <= angle && angle <= M_PI/6) {
                back.push_back((points.at(i)));
            }
            else if (-4*M_PI/6 <= angle && angle <= -2*M_PI/6) {
                left.push_back(points.at(i));
            }
            else if (2*M_PI/6 <= angle && angle <= 4*M_PI/6) {
                right.push_back(points.at(i));
            }
            else if (-5*M_PI/6 >= angle || angle >= 5*M_PI/6) {
                front.push_back(points.at(i));
            }
            else {
                continue;
            }
        }
        /*constexpr float angle_range = M_PI / 4; // 45°

        auto angle_step = (angle_end - angle_start) / points.size();

        for (size_t i = 0; i < points.size(); ++i) {
            auto angle = angle_start + i * angle_step;

            // Normalize angle to [-pi, pi]
            while (angle > M_PI) angle -= 2 * M_PI;
            while (angle < -M_PI) angle += 2 * M_PI;

            // Replace invalid readings
            if (std::isinf(points[i]) || points[i] < 1e-6f) {
                points[i] = range_max;
            }

            float value = points[i];

            // BACK (around 0)
            if (std::abs(angle) <= angle_range) {
                back.push_back(value);
            }
            // LEFT (around -pi/2)
            else if (std::abs(angle + M_PI_2) <= angle_range) {
                left.push_back(value);
            }
            // RIGHT (around +pi/2)
            else if (std::abs(angle - M_PI_2) <= angle_range) {
                right.push_back(value);
            }
            // FRONT (around ±pi)
            else if (std::abs(std::abs(angle) - M_PI) <= angle_range) {
                front.push_back(value);
            }
            else {
                continue;
            }
        }*/

        // TODO: Return the average of each sector (basic mean filter)

        float front_mean = std::accumulate(front.begin(),front.end(),.0) / front.size();
        float back_mean = std::accumulate(back.begin(),back.end(),.0) / back.size();
        float left_mean = std::accumulate(left.begin(),left.end(),.0) / left.size();
        float right_mean = std::accumulate(right.begin(),right.end(),.0) / right.size();


        return LidarFilterResults{
            .front = front_mean,
            .back = back_mean,
            .left = left_mean,
            .right = right_mean
        };
    }
}
