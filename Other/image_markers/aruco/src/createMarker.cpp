#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sstream>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("CreateAruco");
    int marker_id = node->declare_parameter<int>("marker_id", 23);
    cv::Mat markerImage;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::drawMarker(dictionary, marker_id, 200, markerImage, 1);
    
    std::stringstream ss;
    ss << "marker" << marker_id <<".png";
    cv::imwrite(ss.str(), markerImage);
}