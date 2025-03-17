#include "Logic.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

using geometry_msgs::msg::PoseWithCovarianceStamped;

template <>
PoseWithCovarianceStamped fromTransform<PoseWithCovarianceStamped>(const TransformStamped& tf)
{
    PoseWithCovarianceStamped pose;
    //covariance is initialized to all zeros
    pose.header = tf.header;
    pose.pose.pose.position.x = tf.transform.translation.x;
    pose.pose.pose.position.y = tf.transform.translation.y;
    pose.pose.pose.position.z = tf.transform.translation.z;
    pose.pose.pose.orientation = tf.transform.rotation;
    return pose;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    Run<PoseWithCovarianceStamped>();
}