/** ****************************************************************************************
*  Tracking of Aruco tag with a pan-tilt unit
*
* Maintainer: Javier G. Monroy
* MAPIR group: https://mapir.isa.uma.es/
******************************************************************************************** */

#ifndef CrobotStatus_H
#define CrobotStatus_H

#include "rclcpp/rclcpp.hpp"
// ptu d46
#include "ptu_interfaces/srv/set_pan_tilt.hpp"
#include "ptu_interfaces/srv/set_pan_tilt_speed.hpp"
#include "ptu_interfaces/msg/ptu.hpp"
// ptu interbotix wxxms
#include "interbotix_xs_msgs/msg/joint_group_command.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <tf2/exceptions.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <std_msgs/msg/string.hpp>
// boost
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>


class CptuTrack: public rclcpp::Node
{
public:
    CptuTrack();
    ~CptuTrack();    
    void track();

    // Parameters
    std::string tag_frame, ptu_frame, camera_frame;
    std::string ptu_state_topic;
    double loopRate;
    
    // TF2
    std::unique_ptr<tf2_ros::Buffer> tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    std::shared_ptr<tf2_ros::TransformListener> listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // PTU D46 client
    rclcpp::Subscription<ptu_interfaces::msg::PTU>::SharedPtr ptu_sub;
    rclcpp::Client<ptu_interfaces::srv::SetPanTilt>::SharedPtr pan_tilt_client;
    rclcpp::Client<ptu_interfaces::srv::SetPanTiltSpeed>::SharedPtr speed_client;
    std::shared_ptr<ptu_interfaces::srv::SetPanTilt::Request> request = std::make_shared<ptu_interfaces::srv::SetPanTilt::Request>();

    // PTU intebotix
    rclcpp::Publisher<interbotix_xs_msgs::msg::JointGroupCommand>::SharedPtr ptu_interbotix_pub;


protected:
    ptu_interfaces::msg::PTU current_ptu_state;
    void ptuStateCallBack(const ptu_interfaces::msg::PTU::SharedPtr new_state);
    bool initialized;
    bool ptu_d46;
    void ptu_send_pan_tilt(float pan, float tilt);
};

#endif
