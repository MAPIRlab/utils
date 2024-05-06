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

    while(rclcpp::ok())
    {
        rclcpp::spin_some(node);
    }
}

ArucoNode::ArucoNode(std::string name) : rclcpp::Node(name)
{
    std::string imageTopic = declare_parameter<std::string>("imageTopic", "camera/image_raw");
    std::string cameraInfoTopic = declare_parameter<std::string>("cameraInfoTopic", "camera/camera_info");
    markerLength = declare_parameter<float>("markerLength", 0.1);

    imageSub.subscribe(this, imageTopic, rmw_qos_profile_default);
    infoSub.subscribe(this, cameraInfoTopic, rmw_qos_profile_default);

    sync_ = std::make_shared<Synchronizer>(SyncPolicy(1), imageSub, infoSub);
    sync_->registerCallback(std::bind(&ArucoNode::imageCallback, this, std::placeholders::_1, std::placeholders::_2));

    transformBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}


void ArucoNode::imageCallback(const Image::ConstSharedPtr image, const CameraInfo::ConstSharedPtr cameraInfo)
{
    cv_bridge::CvImageConstPtr cvBridgeImage = cv_bridge::toCvShare(image);
    cv::Mat rectified(cvBridgeImage->image.size(), cvBridgeImage->image.type());

    // I guess you just have to know that "k" is the camera matrix and "d" is the dist coefficients
    // great naming, guys
    cv::Mat cameraMatrix(cameraInfo->k, CV_64F);
    cameraMatrix=cameraMatrix.reshape(1,3);
    
    
    cv::undistort(cvBridgeImage->image, rectified, cameraMatrix, cameraInfo->d);
    detectArucoAndPublish(rectified, cameraMatrix, cameraInfo->header);
}

void ArucoNode::detectArucoAndPublish(const cv::Mat& rectifiedImage, const cv::Mat& cameraMatrix, const std_msgs::msg::Header& image_header)
{
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    
    //get the marker corners in image space
    cv::aruco::detectMarkers(rectifiedImage, dictionary, markerCorners, markerIds, detectorParams, rejectedCandidates);

    if(markerCorners.empty())
        return;
    
    //now, let's get the poses from the corners and the camera matrix

    std::array<float, 5> nullDistortion = {0,0,0,0,0}; //image is already rectified
    std::vector<cv::Vec3d> rotations, translations;
    cv::aruco::estimatePoseSingleMarkers(markerCorners, markerLength, cameraMatrix, nullDistortion, rotations, translations);

    for(int i= 0; i< translations.size(); i++)
    {
        geometry_msgs::msg::TransformStamped markerTF;
            markerTF.header=image_header;
            std::stringstream ss;
            ss << "ArucoMarker_" <<i;
            markerTF.child_frame_id = ss.str();
            markerTF.transform.translation.x=translations[i][0];
            markerTF.transform.translation.y=translations[i][1];
            markerTF.transform.translation.z=translations[i][2];
            
            tf2::Vector3 rodriguesVector(rotations[i][0], rotations[i][1], rotations[i][2]);
            tf2::Quaternion q(rodriguesVector.normalized(), rodriguesVector.length());
            markerTF.transform.rotation = tf2::toMsg(q);

        transformBroadcaster->sendTransform(markerTF);
    }
}

