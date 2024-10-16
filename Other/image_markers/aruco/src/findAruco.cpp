#include <aruco/findAruco.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sstream>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArucoNode>("Aruco_node");
    rclcpp::spin(node);
}

ArucoNode::ArucoNode(std::string name) : rclcpp::Node(name)
{
    std::string imageTopic = declare_parameter<std::string>("imageTopic", "camera/image_raw");
    std::string cameraInfoTopic = declare_parameter<std::string>("cameraInfoTopic", "camera/camera_info");
    markerLength = declare_parameter<float>("markerLength", 0.1);

    //imageSub.subscribe(this, imageTopic, rmw_qos_profile_default);
    //infoSub.subscribe(this, cameraInfoTopic, rmw_qos_profile_default);
//
    //SyncPolicy policy(10);
    //policy.setMaxIntervalDuration(rclcpp::Duration(100,0));
    //sync_ = std::make_shared<Synchronizer>(SyncPolicy(policy), imageSub, infoSub);
    //sync_->registerCallback(std::bind(&ArucoNode::imageCallback, this, std::placeholders::_1, std::placeholders::_2));

    imageSub = create_subscription<Image>(imageTopic, 5, std::bind(&ArucoNode::imageCallback, this, std::placeholders::_1));
    cameraInfoSub = create_subscription<CameraInfo>(cameraInfoTopic, 1, std::bind(&ArucoNode::cameraInfoCallback, this, std::placeholders::_1));
    transformBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    detectionsPub = create_publisher<image_marker_msgs::msg::MarkerDetection>("aruco/detections", 20);
}

void ArucoNode::cameraInfoCallback(const CameraInfo::ConstSharedPtr cameraInfo)
{
    m_cameraInfo = cameraInfo;
}


void ArucoNode::imageCallback(const Image::ConstSharedPtr image)
{
    RCLCPP_INFO(get_logger(), "Got a new image");
    if(!m_cameraInfo)
    {
        RCLCPP_WARN(get_logger(), "No camera info yet");
    }
    cv_bridge::CvImageConstPtr cvBridgeImage = cv_bridge::toCvShare(image);

    // I guess you just have to know that "k" is the camera matrix and "d" is the dist coefficients
    // great naming, guys
    cv::Mat cameraMatrix(m_cameraInfo->k, CV_64F);
    cameraMatrix = cameraMatrix.reshape(1, 3);
    detectArucoAndPublish(cvBridgeImage->image, cameraMatrix, m_cameraInfo->d, m_cameraInfo->header);
    RCLCPP_INFO(get_logger(), "Ending callback");
}

void ArucoNode::detectArucoAndPublish(const cv::Mat& image, const cv::Mat& cameraMatrix, const std::vector<double>& distCoeffs, const std_msgs::msg::Header& image_header)
{
    static cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
    static cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    // get the marker corners in image space
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds, detectorParams, rejectedCandidates);

    RCLCPP_INFO(get_logger(), "Detected %lu markers in image", markerCorners.size());

    if (markerCorners.empty())
        return;

    // now, let's get the poses from the corners and the camera matrix

    std::vector<cv::Vec3d> rotations, translations;
    cv::aruco::estimatePoseSingleMarkers(markerCorners, markerLength, cameraMatrix, distCoeffs, rotations, translations);

    // re-project the centers to image space
    std::vector<cv::Point2d> imagePoints;
    // no translation or rotation, since the points are already relative to the camera's reference system
    cv::projectPoints(translations, std::array<float, 3>{0, 0, 0}, std::array<float, 3>{0, 0, 0}, cameraMatrix, distCoeffs, imagePoints);

    for (int i = 0; i < translations.size(); i++)
    {
        // publish TF
        geometry_msgs::msg::TransformStamped markerTF;
        markerTF.header = image_header;
        std::stringstream ss;
        ss << "ArucoMarker_" << markerIds[i];
        markerTF.child_frame_id = ss.str();
        markerTF.transform.translation.x = translations[i][0];
        markerTF.transform.translation.y = translations[i][1];
        markerTF.transform.translation.z = translations[i][2];

        tf2::Vector3 rodriguesVector(rotations[i][0], rotations[i][1], rotations[i][2]);
        tf2::Quaternion q(rodriguesVector.normalized(), rodriguesVector.length());
        markerTF.transform.rotation = tf2::toMsg(q);

        transformBroadcaster->sendTransform(markerTF);

        // publish message

        image_marker_msgs::msg::MarkerDetection detectionsMsg;
        detectionsMsg.stamp = now();
        detectionsMsg.marker_type = "Aruco";
        detectionsMsg.id = markerIds[i];
        detectionsMsg.transform = markerTF;
        detectionsMsg.camera_space_point.x = imagePoints[i].x;
        detectionsMsg.camera_space_point.y = imagePoints[i].y;

        detectionsPub->publish(detectionsMsg);
    }
}
