/** ****************************************************************************************
 *  Tracking of Aruco tag with a pan-tilt unit
 *
 * Maintainer: Javier G. Monroy
 * MAPIR group: https://mapir.isa.uma.es/
 ******************************************************************************************** */

#include "ptu_tracking/ptu_tracking.h"
#include <sstream>
#include <stdlib.h> /* atoi */
#include <vector>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2/exceptions.h"
#include "tf2/utils.h"
#include <math.h>

using namespace std;
using std::placeholders::_1;

// --------------------------------------------
// CptuTrack
//---------------------------------------------

CptuTrack::CptuTrack() : Node("CptuTrack")
{
    // Read Parameters
    //----------------
    tag_frame = this->declare_parameter<std::string>("tag_frame_id", "landmark_link");
    ptu_frame = this->declare_parameter<std::string>("ptu_frame_id", "ptu_link");

    tag_detection_topic = this->declare_parameter<std::string>("tag_detection_topic", "apriltag/detections");
    image_size_x = this->declare_parameter<int>("image_size_x", 1920);
    image_size_y = this->declare_parameter<int>("image_size_y", 1080);
    kp = this->declare_parameter<float>("kp", 1);
    ki = this->declare_parameter<float>("ki", 1);
    kd = this->declare_parameter<float>("kd", 1);

    tf_based_tracking = this->declare_parameter<bool>("tf_based_tracking", true);
        
    // Publisher
    ptu_interbotix_pub = this->create_publisher<interbotix_xs_msgs::msg::JointGroupCommand>("commands/joint_group", 1);

    // Subscriber?
    apriltag_sub = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(tag_detection_topic,1,std::bind(&CptuTrack::detected_tag_cb,this, _1) );
    
    // PID controller
    pid_controller = std::make_shared<PID>(this->get_clock(), kp, ki, kd);

    // check PTU movement
    RCLCPP_INFO(this->get_logger(), "checking PTU conectivity....");
    sleep(1.0);
    ptu_send_pan_tilt(-0.1, 0.0);
    sleep(0.5);
    ptu_send_pan_tilt(0.1, 0.);
    sleep(0.5);
    ptu_send_pan_tilt(0.0, 0.0);

    // Set initial state
    current_ptu_pan = 0.0;
    current_ptu_tilt = 0.0;
    sleep(0.5);

    initialized = true;
    RCLCPP_INFO(this->get_logger(), "Ready for Tracking....");
}


CptuTrack::~CptuTrack()
{
    // check movement
    RCLCPP_INFO(this->get_logger(), "Stopping PTU in (0,0)...");
    ptu_send_pan_tilt(0.0, 0.0);
    sleep(1.0);
    RCLCPP_INFO(this->get_logger(), "See you later, aligator!");
}


void CptuTrack::detected_tag_cb(apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
{
    // The tag has been detected -> Track
    if (!initialized)
    {
        RCLCPP_WARN(this->get_logger(), "PTU state unknown... waiting");
        return;
    }

    // Select tracking method
    if (tf_based_tracking)
    {
        RCLCPP_INFO(this->get_logger(), "Tag Detected --> tracking by TF");
        do_tf_based_tracking();
    }
    else
    {
        // Detection is given in image frame (px)
        if ( msg->detections.size() > 0)
        {
            float tag_x = msg->detections[0].centre.x;
            float tag_y = msg->detections[0].centre.y;
            RCLCPP_INFO(this->get_logger(), "Tag Detected --> tracking on IMG X[%.2f] Y[%.2f]",tag_x,tag_y);
            do_image_based_tracking(tag_x,tag_y);
        }
    }
}


void CptuTrack::ptu_send_pan_tilt(float pan, float tilt)
{
    //RCLCPP_INFO(this->get_logger(), "Requesting pan[%.2f] & tilt[%.2f]", pan, tilt);

    // PTU interbotix: Command via topi
    interbotix_xs_msgs::msg::JointGroupCommand message;
    message.name = "turret";
    message.cmd.clear();
    message.cmd.push_back(pan);
    message.cmd.push_back(tilt);
    ptu_interbotix_pub->publish(message);    
}


void CptuTrack::do_image_based_tracking(float tag_x, float tag_y)
{
    //RCLCPP_INFO(this->get_logger(), "IN");

    float precission_px = 5.0;   // px
    float step_rad = 0.001;       // rad
    float error_x = tag_x - float(image_size_x)/2;
    float error_y = tag_y - float(image_size_y)/2;
    RCLCPP_INFO(this->get_logger(), "TAG distance (px) with respect ImgCenter X[%.2f] Y[%.2f]", error_x, error_y);

    
    // pan
    if (abs(error_y) > precission_px)
    {
        float incr_rad = -pid_controller->DoUpdate(error_y*step_rad);
        RCLCPP_INFO(this->get_logger(), "PAN increment [%.4f] rad", incr_rad);
        current_ptu_pan += incr_rad;
    }
    // tilt
    if (abs(error_x) > precission_px)
    {
        float incr_rad = -pid_controller->DoUpdate(error_x*step_rad);
        RCLCPP_INFO(this->get_logger(), "TILT increment [%.4f] rad", incr_rad);
        current_ptu_tilt += incr_rad;
    }
    // set goal pan in one step
    ptu_send_pan_tilt(current_ptu_pan, current_ptu_tilt);
}


void CptuTrack::do_tf_based_tracking()
{
    // 1. Get tag pose with respect PTU
    geometry_msgs::msg::TransformStamped transform_tag;
    try
    {
        transform_tag = tf_buffer->lookupTransform(ptu_frame, tag_frame, tf2::TimePointZero, 20ms);
    }
    catch (tf2::TransformException& ex)
    {
        RCLCPP_WARN(this->get_logger(), "Exception while reading tf transforms -------> %s", ex.what());
        RCLCPP_WARN(this->get_logger(), "TAG not in camera FOV");
        return;
    }
    double tag_x = transform_tag.transform.translation.x;
    double tag_y = transform_tag.transform.translation.y;
    double tag_z = transform_tag.transform.translation.z;
    RCLCPP_INFO(this->get_logger(), "TAG DETECTED x[%.2f], y[%.2f], z[%.2f] with respect PTU frame", tag_x, tag_y, tag_z);

    bool one_step = false;
    if (one_step)
    {
        // 2.Estimate pan&tilt (absolute values using depth estimation)
        double tag_pan = atan2(tag_y, tag_x);
        double tag_tilt = -atan2(tag_z, tag_x);        

        // set goal pan&tily in one step
        ptu_send_pan_tilt(tag_pan, tag_tilt);
    }
    else
    {
        // Command incrementally
        double precission = 0.05;   // m
        double step_rad = 0.001;    // rad
                
        // pan
        if (abs(tag_y) > precission)
        {
            current_ptu_pan += kp*tag_y*step_rad;
        }
        //tilt
        if (abs(tag_z) > precission)
        {
            current_ptu_tilt -= kp*tag_z*step_rad;
        }
        /*
        if (tag_y > precission)
            current_ptu_pan += step_rad; // rad (0.57deg)
        else if (tag_y < -precission)
            current_ptu_pan -= step_rad; // rad

        // tilt
        if (tag_z > precission)
            current_ptu_tilt -= step_rad; // rad
        else if (tag_z < -precission)
            current_ptu_tilt += step_rad; // rad
        */
        // set goal pan in one step
        ptu_send_pan_tilt(current_ptu_pan, current_ptu_tilt);
    }
}


//-----------------------------------------------------------------------------------
//                                   MAIN
//-----------------------------------------------------------------------------------
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CptuTrack>();

    // Main Loop
    //----------
    RCLCPP_INFO(node->get_logger(), "CptuTrack ready for operation...Looping");
    rclcpp::spin(node);

    return (0);
}
