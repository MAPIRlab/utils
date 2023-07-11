/** ****************************************************************************************
*  Tracking of Aruco tag with a pan-tilt unit
*
* Maintainer: Javier G. Monroy
* MAPIR group: https://mapir.isa.uma.es/
******************************************************************************************** */

#include "ptu_tracking/ptu_tracking.h"
#include <sstream>
#include <stdlib.h>     /* atoi */
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
    camera_frame = this->declare_parameter<std::string>("camera_frame_id", "camera_link");
    ptu_frame = this->declare_parameter<std::string>("ptu_frame_id", "ptu_link");
    loopRate = this->declare_parameter<double>("loopRate", 1.0);
    ptu_state_topic = this->declare_parameter<std::string>("ptu_state_topic", "/ptu/state");
        
    // Service Clients for PTU
    //------------------------
    ptu_sub = this->create_subscription<ptu_interfaces::msg::PTU>(ptu_state_topic, 1, std::bind(&CptuTrack::ptuStateCallBack, this, _1));
    pan_tilt_client = this->create_client<ptu_interfaces::srv::SetPanTilt>("/ptu/set_pan_tilt");  
    speed_client = this->create_client<ptu_interfaces::srv::SetPanTiltSpeed>("/ptu/set_pan_tilt_speed");  

    // Wait for pan&tilt (srv)
    while (!pan_tilt_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not yet available, waiting again...");
    }
    
    // Test PTU
    auto request_speed = std::make_shared<ptu_interfaces::srv::SetPanTiltSpeed::Request>();
    request_speed->pan_speed = 1.5;
    request_speed->tilt_speed = 1.5;
    auto result_speed = speed_client->async_send_request(request_speed);

    // check movement
    request->pan = 2.0;
    request->tilt = 0.1;
    auto result = pan_tilt_client->async_send_request(request);
    sleep(2.0);
   
    // set (0,0)
    request->pan = 0.0;
    request->tilt = 0.0;
    result = pan_tilt_client->async_send_request(request);

    initialized = false;
    RCLCPP_INFO(this->get_logger(),"Ready for Tracking....");
}


CptuTrack::~CptuTrack()
{
}


void CptuTrack::ptuStateCallBack(const ptu_interfaces::msg::PTU::SharedPtr new_state)
{
    // update ptu status
    current_ptu_state = *new_state;
    RCLCPP_INFO(this->get_logger(),"New ptu pose received: pan[%.2f], tilt[%.2f]", new_state->pan, new_state->tilt);
    initialized = true;
}


void CptuTrack::track()
{
    if (!initialized)
    {
         RCLCPP_WARN(this->get_logger(),"PTU state unknown... waiting");
        return;
    }


    // 1. Get tag and camera poses with respect ptu
    geometry_msgs::msg::TransformStamped transform_tag, transform_camera;
    try 
    {
        transform_tag = tf_buffer->lookupTransform(ptu_frame, tag_frame, tf2::TimePointZero,100ms);
        transform_camera = tf_buffer->lookupTransform(ptu_frame, camera_frame, tf2::TimePointZero,100ms);
    } 
    catch(tf2::TransformException &ex) 
    {
        RCLCPP_WARN(this->get_logger(),"Exception while reading tf transforms -------> %s", ex.what());
        return;
    }

    // 2.Estimate pan&tilt 
    double tag_x = transform_tag.transform.translation.x;
    double tag_y = transform_tag.transform.translation.y;
    double tag_z = transform_tag.transform.translation.z;

    double goal_pan = atan2(tag_y, tag_x);
    
    // Command incrementally
    if (abs(goal_pan - current_ptu_state.pan) > 0.5*M_PI/180) 
    {
        if (goal_pan - current_ptu_state.pan < 0)
        {
            // pan-negative
            request->pan = current_ptu_state.pan - M_PI/180;
            RCLCPP_INFO(this->get_logger(),"Requesting (-)Pan: goal_pan[%.2f], current_pan[%.2f]", goal_pan, current_ptu_state.pan);
        }
        else
        {
            //pan-positive
            request->pan = current_ptu_state.pan + M_PI/180;
            RCLCPP_INFO(this->get_logger(),"Requesting (+)Pan: goal_pan[%.2f], current_pan[%.2f]", goal_pan, current_ptu_state.pan);
        }

        // Set pan&tilt (srv request)
        auto result = pan_tilt_client->async_send_request(request);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(),"Camera is aligned with Tag... doing nothing!");
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
    RCLCPP_INFO(node->get_logger(),"CptuTrack ready for operation...Looping");
    rclcpp::Rate loop_rate(node->loopRate);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);        //Check for new msgs (callbacks)
        node->track();
        loop_rate.sleep();
    }

    return(0);
}
