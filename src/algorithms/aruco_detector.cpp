#include "aruco_detector.hpp"


namespace algorithms
{
   ArucoDetector::ArucoDetector() { 
            dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
   }

   std::vector<ArucoDetector::Aruco> ArucoDetector::detect(cv::Mat frame) {
            std::vector<Aruco> arucos;

            std::vector<int> marker_ids;
            std::vector<std::vector<cv::Point2f>> marker_corners;

            cv::aruco::detectMarkers(frame, dictionary_, marker_corners, marker_ids);

            if (!marker_ids.empty()) {
                std::cout << "Arucos found: ";
                for (size_t i = 0; i < marker_ids.size(); i++) {
                    std::cout << marker_ids[i] << " ";

                    arucos.emplace_back(Aruco{marker_ids[i], marker_corners[i]});
                }
                std::cout << std::endl;
            }

            return arucos;
        }

} // namespace algorithms
