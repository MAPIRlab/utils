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
    auto sub = node->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 1, callback);
    
    rclcpp::spin(node);
}