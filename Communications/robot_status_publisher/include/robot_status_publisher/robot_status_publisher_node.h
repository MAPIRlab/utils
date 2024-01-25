/** ****************************************************************************************
 *  This node implements a simple publisher of the current robot status
 *  Designed to serve as a monitor of the robot status.
 *
 * Maintainer: Javier G. Monroy
 * MAPIR group: https://mapir.isa.uma.es/
 ******************************************************************************************** */

#ifndef CrobotStatus_H
#define CrobotStatus_H

#include "rclcpp/rclcpp.hpp"
 // ros2 msgs
#include <diagnostic_msgs/msg/key_value.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <tf2/exceptions.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/string.hpp>
// boost
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>
// json
#include <mqtt_serialization/PoseJSON.hpp>

class CrobotStatus : public rclcpp::Node
{
public:
    CrobotStatus();
    ~CrobotStatus();
    void sendStatus();

    // Parameters
    std::string output_topic; // defaults to ros2mqtt
    double statusRate;

    std::string map_frame, base_frame;
    std::string tf_prefix;
    std::string battery_topic;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    std::shared_ptr<tf2_ros::TransformListener> listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

protected:
    // Topic Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_sub;

    // Publisher (status as JSON)
    rclcpp::Publisher<diagnostic_msgs::msg::KeyValue>::SharedPtr status_pub;

    // Msgs To report
    geometry_msgs::msg::PoseWithCovarianceStamped current_pose;
    sensor_msgs::msg::BatteryState current_battery;
    std_msgs::msg::String current_task;

    // CallBacks
    void batteryCallBack(const sensor_msgs::msg::BatteryState::SharedPtr new_battery);
    void runningTaskCallBack(const std_msgs::msg::String::SharedPtr new_running_task);
};

#endif
