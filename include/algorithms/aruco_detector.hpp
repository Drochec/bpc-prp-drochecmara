#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

namespace algorithms {

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