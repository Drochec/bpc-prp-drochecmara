#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

namespace algorithms {

    enum class ArucoID {
        NONE = -1,
        STRAIGHT = 0,
        LEFT = 1,
        RIGHT = 2,
        TREASURE_STRAIGHT = 10,
        TREASURE_RIGHT = 11,
        TREASURE_LEFT = 12,
    };

    class ArucoDetector {
    public:

        // Represents one detected marker
        struct Aruco {
            int id;
            std::vector<cv::Point2f> corners;
        };

        ArucoDetector();

        ~ArucoDetector() = default;

        // Detect markers in the input image
        std::vector<Aruco> detect(cv::Mat frame);
        
    private:
        cv::Ptr<cv::aruco::Dictionary> dictionary_;
    };
}