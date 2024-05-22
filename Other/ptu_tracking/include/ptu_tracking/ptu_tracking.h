/** ****************************************************************************************
 *  Tracking of Aruco tag with a pan-tilt unit
 *
 * Maintainer: Javier G. Monroy
 * MAPIR group: https://mapir.isa.uma.es/
 ******************************************************************************************** */

#ifndef CrobotStatus_H
#define CrobotStatus_H

#include "rclcpp/rclcpp.hpp" 
#include "ptu_tracking/PID.h"

 // ptu interbotix wxxms
#include "interbotix_xs_msgs/msg/joint_group_command.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <tf2/exceptions.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <std_msgs/msg/string.hpp>
#include <image_marker_msgs/msg/marker_detection.hpp>
// boost
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>

class CptuTrack : public rclcpp::Node
{
public:
    CptuTrack();
    ~CptuTrack();
    void do_tracking();

    // Parameters
    int image_size_x, image_size_y;
    int goal_marker_x, goal_marker_y;
    std::string tag_frame, ptu_frame, tag_detection_topic;
    float kp_p,ki_p,kd_p;
    float kp_t,ki_t,kd_t;
    bool tf_based_tracking;

    // TF2
    std::unique_ptr<tf2_ros::Buffer> tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    std::shared_ptr<tf2_ros::TransformListener> listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // PID controller
    std::shared_ptr<PID> pid_controller_pan;
    std::shared_ptr<PID> pid_controller_tilt;
    
    // PTU intebotix
    rclcpp::Publisher<interbotix_xs_msgs::msg::JointGroupCommand>::SharedPtr ptu_interbotix_pub;

    // Aruco Detections (see image_marker_msgs)
    rclcpp::Subscription<image_marker_msgs::msg::MarkerDetection>::SharedPtr aruco_sub;

#if USE_GUI
    void renderGUI();
#endif

protected:
    bool initialized;
    double current_ptu_pan, current_ptu_tilt;
    float last_detected_tag_x;
    float last_detected_tag_y;
    double previousTimestamp;
    
    void do_tf_based_tracking();
    void do_image_based_tracking(float tag_x, float tag_y);
    void ptu_send_pan_tilt(float pan, float tilt);
    void detected_tag_cb(image_marker_msgs::msg::MarkerDetection::SharedPtr msg);


};

#endif
