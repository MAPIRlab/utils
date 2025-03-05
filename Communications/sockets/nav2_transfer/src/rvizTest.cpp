#include "nav2_msgs/action/navigate_to_pose.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/client.hpp>
#include <rclcpp_action/create_client.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("test");
    auto client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node, "navigate_to_pose");

    auto callback = [&](const geometry_msgs::msg::PoseStamped::SharedPtr pose)
    {
        RCLCPP_INFO(rclcpp::get_logger("TestNode"), "Received message, sending goal to action server");
        nav2_msgs::action::NavigateToPose::Goal goal;
        goal.pose = *pose;

        client->async_send_goal(goal);
    };
    std::string topic = node->declare_parameter<std::string>("topic", "/goal_pose");
    auto sub = node->create_subscription<geometry_msgs::msg::PoseStamped>("topic", 1, callback);
    RCLCPP_INFO(node->get_logger(), "Listening to topic '%s", sub->get_topic_name());
    
    rclcpp::spin(node);
}