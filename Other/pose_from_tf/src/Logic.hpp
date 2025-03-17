#pragma once
#include "BufferWrapper.hpp"
using geometry_msgs::msg::TransformStamped;

template <typename PoseT>
PoseT fromTransform(const TransformStamped& tf);

template <typename PoseT>
void Run()
{
    auto node = std::make_shared<rclcpp::Node>("pose_tf");
    BufferWrapper buf(node->get_clock());

    std::string frame_id = node->declare_parameter("frame_id", "base_link");

    std::string topic = node->declare_parameter("topic", "/pose_tf");
    std::shared_ptr<rclcpp::Publisher<PoseT>> pub = node->create_publisher<PoseT>(topic, 5);

    double freq = node->declare_parameter("frequency", 5.0);

    RCLCPP_INFO(node->get_logger(), "Publishing tf '%s' as pose on topic '%s' at %.1fHz", frame_id.c_str(), pub->get_topic_name(), freq);
    rclcpp::Rate rate(freq);
    while (rclcpp::ok())
    {
        try
        {
            TransformStamped transform = buf.buffer.lookupTransform("map", frame_id, rclcpp::Time(0));
            PoseT pose = fromTransform<PoseT>(transform);
            pub->publish(pose);
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(node->get_logger(), "Exception: '%s'", e.what());
        }
        rate.sleep();
    }
}