#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("nav2_mqtt_Test_node");

    auto client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose> (node, "nav2MQTT");

    nav2_msgs::action::NavigateToPose::Goal goal;

    goal.pose.header.frame_id = "map";
    goal.pose.pose.position.x=0;
    goal.pose.pose.position.y=0;
    goal.pose.pose.position.z=0;

    client->async_send_goal(goal);
    return 0;
}