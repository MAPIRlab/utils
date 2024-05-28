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

    tag_detection_topic = this->declare_parameter<std::string>("tag_detection_topic", "aruco/detections");
    image_size_x = this->declare_parameter<int>("image_size_x", 1920);
    image_size_y = this->declare_parameter<int>("image_size_y", 1080);
    goal_marker_x = this->declare_parameter<int>("goal_marker_x", image_size_x/2);
    goal_marker_y = this->declare_parameter<int>("goal_marker_y", image_size_y/2);
    // PID
    kp_p = this->declare_parameter<float>("kp_p", 1);
    ki_p = this->declare_parameter<float>("ki_p", 1);
    kd_p = this->declare_parameter<float>("kd_p", 1);
    int window_size_p = this->declare_parameter<int>("window_size_p", 10);


    kp_t = this->declare_parameter<float>("kp_t", 1);
    ki_t = this->declare_parameter<float>("ki_t", 1);
    kd_t = this->declare_parameter<float>("kd_t", 1);
    int window_size_t = this->declare_parameter<int>("window_size_t", 10);

    tf_based_tracking = this->declare_parameter<bool>("tf_based_tracking", true);
        
    // Publisher
    ptu_interbotix_pub = this->create_publisher<interbotix_xs_msgs::msg::JointGroupCommand>("commands/joint_group", 1);

    // Subscriber?
    aruco_sub = this->create_subscription<image_marker_msgs::msg::MarkerDetection>(tag_detection_topic,1,std::bind(&CptuTrack::detected_tag_cb,this, _1) );
    
    // PID controller
    pid_controller_pan = std::make_shared<PID>(this->get_clock(), kp_p, ki_p, kd_p, window_size_p);
    pid_controller_tilt = std::make_shared<PID>(this->get_clock(), kp_t, ki_t, kd_t, window_size_t);
    

    // check PTU movement
    RCLCPP_INFO(this->get_logger(), "checking PTU conectivity....");
    previousTimestamp = this->get_clock()->now().seconds();
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


void CptuTrack::do_tracking()
{
    // Track the last seen marker (if not too old)
    if (!initialized)
    {
        RCLCPP_WARN(this->get_logger(), "PTU state unknown... waiting");
        return;
    }

    // Check we have fresh detections!
    double time_diff = this->get_clock()->now().seconds() - previousTimestamp;
    if ( time_diff > 0.5)
    {
        //RCLCPP_INFO(this->get_logger(), "Not recent marker detected... doing Nothing (time_diff=%f)!!",time_diff);
        return;
    }

    // Select tracking method
    if (tf_based_tracking)
    {
        RCLCPP_INFO(this->get_logger(), "tracking by TF");
        do_tf_based_tracking();
    }
    else
    {
        //RCLCPP_INFO(this->get_logger(), "tracking on IMG");
        // Detection is given in image frame (px)
        do_image_based_tracking(last_detected_tag_x,last_detected_tag_y);   
    }
}
/*
void CptuTrack::detected_tag_cb(image_marker_msgs::msg::MarkerDetection::SharedPtr msg)
{
    // Update detection values
    last_detected_tag_x = msg->camera_space_point.x;
    last_detected_tag_y = msg->camera_space_point.y;
    previousTimestamp = this->get_clock()->now().seconds();
    //RCLCPP_INFO(this->get_logger(), "Tag Detected at X[%.2f] Y[%.2f]",last_detected_tag_x,last_detected_tag_y);
}
*/


void CptuTrack::detected_tag_cb(image_marker_msgs::msg::MarkerDetection::SharedPtr msg)
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
        float tag_x = msg->camera_space_point.x;
        float tag_y = msg->camera_space_point.y;
        //RCLCPP_INFO(this->get_logger(), "Tag Detected --> tracking on IMG X[%.2f] Y[%.2f]",tag_x,tag_y);
        do_image_based_tracking(tag_x,tag_y);   
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
    float error_x = goal_marker_x - tag_x;
    float error_y = goal_marker_y - tag_y;
    //RCLCPP_INFO(this->get_logger(), "TAG distance (error in px) with respect Goal: X[%.2f] Y[%.2f]", error_x, error_y);

    
    // pan
    if (abs(error_y) > precission_px)
    {
        float incr_rad = pid_controller_pan->DoUpdate(error_y * 0.0001);
        //RCLCPP_INFO(this->get_logger(), "PAN increment [%.4f] rad", incr_rad);
        current_ptu_pan += incr_rad;

        // Oscilation freq
        auto sign = [](float f){return f>0?1:-1;};
        static float previous = 0;
        if(previous != 0)
        {
            if(sign(previous) != sign(incr_rad))
                RCLCPP_WARN(get_logger(), "Changed directions at %f", now().seconds());
        }
        previous = incr_rad;
    }
    // tilt
    if (abs(error_x) > precission_px)
    {
        float incr_rad = pid_controller_tilt->DoUpdate(error_x* 0.0001);
        //RCLCPP_INFO(this->get_logger(), "TILT increment [%.4f] rad", incr_rad);
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
            current_ptu_pan += kp_p*tag_y*step_rad;
        }
        //tilt
        if (abs(tag_z) > precission)
        {
            current_ptu_tilt -= kp_p*tag_z*step_rad;
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
#define USE_GUI 1
#if USE_GUI
    #include <ptu_tracking/PID_GUI.h>
    #include <thread>
    #include <ament_index_cpp/get_package_share_directory.hpp>
    void CptuTrack::renderGUI()
    {
        std::string iniFilePath = ament_index_cpp::get_package_share_directory("ptu_tracking")+"/resources/imgui.ini";
        AmentImgui::Setup(iniFilePath.c_str(), "PID GUI");
        rclcpp::Rate r(30);
        while(rclcpp::ok())
        {
            AmentImgui::StartFrame();
            RenderPIDGUI(*pid_controller_pan, "Pan");
            RenderPIDGUI(*pid_controller_tilt, "Tilt");
            ImGui::Begin("Goal pixels");
            ImGui::InputInt("goal_marker_x", &goal_marker_x);
            ImGui::InputInt("goal_marker_y", &goal_marker_y);
            ImGui::End();
            AmentImgui::Render();
        }
        AmentImgui::Close();
    }
#endif

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

#if USE_GUI
    std::jthread renderThread(std::bind(&CptuTrack::renderGUI, node));
#endif

    rclcpp::spin(node);
    //rclcpp::Rate loop_rate(30.0);
    //while (rclcpp::ok()) 
    //{
    //    rclcpp::spin_some(node);
    //    node->do_tracking();
    //    loop_rate.sleep();
    //}

    return (0);
}
