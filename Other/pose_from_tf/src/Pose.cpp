#include "Logic.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

using geometry_msgs::msg::PoseStamped;

template <>
PoseStamped fromTransform<PoseStamped>(const TransformStamped& tf)
{
    PoseStamped pose;
    pose.header = tf.header;
    pose.pose.position.x = tf.transform.translation.x;
    pose.pose.position.y = tf.transform.translation.y;
    pose.pose.position.z = tf.transform.translation.z;
    pose.pose.orientation = tf.transform.rotation;
    return pose;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    Run<PoseStamped>();
}