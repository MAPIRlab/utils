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
    ptu_d46 = false;

    // Service Clients for PTU
    //------------------------    
    if (ptu_d46)
    {
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

        // Set speed
        auto request_speed = std::make_shared<ptu_interfaces::srv::SetPanTiltSpeed::Request>();
        request_speed->pan_speed = 1.5;
        request_speed->tilt_speed = 1.5;
        auto result_speed = speed_client->async_send_request(request_speed);
    }
    else
    {
        ptu_interbotix_pub = this->create_publisher<interbotix_xs_msgs::msg::JointGroupCommand>("commands/joint_group",1);
    }
    
    // check movement
    RCLCPP_INFO(this->get_logger(),"checking PTU conectivity....");
    sleep(5.0);
    ptu_send_pan_tilt(1.0, 0.2);     
    sleep(3.0);
    ptu_send_pan_tilt(-1.0, -0.2);
    sleep(3.0);
    ptu_send_pan_tilt(0.0, 0.0);
    sleep(3.0);
   
    initialized = true;
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

void CptuTrack::ptu_send_pan_tilt(float pan, float tilt)
{

    RCLCPP_INFO(this->get_logger(),"Requesting pan[%.2f] & tilt[%.2f]", pan, tilt);
    if (ptu_d46)
    {
        // PTU D46: Command via service call
        request->pan = pan;
        request->tilt = tilt;
        auto result = pan_tilt_client->async_send_request(request);
    }
    else
    {
        //PTU interbotix: Command via topi
        interbotix_xs_msgs::msg::JointGroupCommand message;
        message.name = "turret";
        message.cmd.clear();
        message.cmd.push_back(pan);
        message.cmd.push_back(tilt);
        ptu_interbotix_pub->publish(message);
    }
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
        transform_tag = tf_buffer->lookupTransform(ptu_frame, tag_frame, tf2::TimePointZero,20ms);
        transform_camera = tf_buffer->lookupTransform(ptu_frame, camera_frame, tf2::TimePointZero,200ms);
    } 
    catch(tf2::TransformException &ex) 
    {
        RCLCPP_WARN(this->get_logger(),"Exception while reading tf transforms -------> %s", ex.what());
        RCLCPP_WARN(this->get_logger(),"TAG not in camera FOV");
        return;
    }

    // 2.Estimate pan&tilt 
    double tag_x = transform_tag.transform.translation.x;
    double tag_y = transform_tag.transform.translation.y;
    double tag_z = transform_tag.transform.translation.z;
    double tag_pan = atan2(tag_y, tag_x);
    double tag_tilt = -atan2(tag_z, tag_x);
    
    double camera_x = transform_camera.transform.translation.x;
    double camera_y = transform_camera.transform.translation.y;
    double camera_z = transform_camera.transform.translation.z;
    double camera_pan = atan2(camera_y, camera_x);
    RCLCPP_INFO(this->get_logger(),"tag_pan[%.2f], camera_pan[%.2f]", tag_pan, camera_pan);


    bool one_step = true;

    if (one_step)
    {
        // set goal pan in one step
        ptu_send_pan_tilt(tag_pan, tag_tilt);
    }    
    else 
    {
        // Command incrementally
        if (abs(tag_pan - current_ptu_state.pan) > 0.5*M_PI/180) 
        {
            if (tag_pan - current_ptu_state.pan < 0)
            {
                // pan-negative
                request->pan = current_ptu_state.pan - M_PI/180;
                RCLCPP_INFO(this->get_logger(),"Requesting (-)Pan: tag_pan[%.2f], current_pan[%.2f]", tag_pan, current_ptu_state.pan);
            }
            else
            {
                //pan-positive
                request->pan = current_ptu_state.pan + M_PI/180;
                RCLCPP_INFO(this->get_logger(),"Requesting (+)Pan: tag_pan[%.2f], current_pan[%.2f]", tag_pan, current_ptu_state.pan);
            }

            // Set pan&tilt (srv request)
            auto result = pan_tilt_client->async_send_request(request);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(),"Camera is aligned with Tag... doing nothing!");
        }
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
