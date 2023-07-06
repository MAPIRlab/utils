#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("nav2_mqtt_Test_node");

    std::string action_name = node->declare_parameter<std::string>("action_name", "nav2MQTT");
    auto client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose> (node, action_name);
    
    using namespace std::chrono_literals;
    rclcpp::sleep_for(std::chrono::nanoseconds(3s));

    RCLCPP_INFO(node->get_logger(), "Sending action to: %s", action_name.c_str() );

    nav2_msgs::action::NavigateToPose::Goal goal;

    goal.pose.header.frame_id = "map";
    goal.pose.pose.position.x=node->declare_parameter<float>("x_goal");
    goal.pose.pose.position.y=node->declare_parameter<float>("y_goal");
    goal.pose.pose.position.z=0;

    client->async_send_goal(goal);
    return 0;
}