#pragma once 

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <image_marker_msgs/msg/marker_detection.hpp>

#include <opencv2/core.hpp>

using Image = sensor_msgs::msg::Image;
using CameraInfo = sensor_msgs::msg::CameraInfo;

class ArucoNode : public rclcpp::Node
{
public:
    ArucoNode(std::string name);
private:
    float markerLength; //length of the side of the marker (meters)

    CameraInfo::ConstSharedPtr m_cameraInfo;
    
    //message_filters::Subscriber<Image> imageSub;
    //message_filters::Subscriber<CameraInfo> infoSub;
    //using SyncPolicy = message_filters::sync_policies::ApproximateTime<Image, CameraInfo>;
    //using Synchronizer = message_filters::Synchronizer<SyncPolicy>;
    //std::shared_ptr<Synchronizer> sync_;

    rclcpp::Subscription<Image>::SharedPtr imageSub;
    rclcpp::Subscription<CameraInfo>::SharedPtr cameraInfoSub;

    rclcpp::Publisher<image_marker_msgs::msg::MarkerDetection>::SharedPtr detectionsPub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> transformBroadcaster;

    void cameraInfoCallback(const CameraInfo::ConstSharedPtr cameraInfo);
    void imageCallback(const Image::ConstSharedPtr image);
    void detectArucoAndPublish(const cv::Mat& image, const cv::Mat& cameraMatrix, const std::vector<double>& distCoeffs, const std_msgs::msg::Header& image_header);
};