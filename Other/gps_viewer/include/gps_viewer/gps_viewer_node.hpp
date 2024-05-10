#pragma once
#include <rclcpp/rclcpp.hpp>
#include "mainwindow.hpp"
#include <sensor_msgs/msg/nav_sat_fix.hpp>

class GPSViewerNode : public rclcpp::Node
{
public:
    GPSViewerNode(std::string name);

private:
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gpsSub;
    MainWindow window;

    void gpsCallback(sensor_msgs::msg::NavSatFix::ConstSharedPtr msg);
};