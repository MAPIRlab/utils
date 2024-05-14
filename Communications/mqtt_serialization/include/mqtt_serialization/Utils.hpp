#pragma once
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace mqtt_serialization
{

    namespace Utils
    {
        inline double getYaw(const geometry_msgs::msg::Quaternion& quat)
        {
            tf2::Quaternion tfquat;
            tf2::fromMsg(quat, tfquat);

            tf2::Matrix3x3 m(tfquat);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            return yaw;
        }

        inline std::string applyNamespaceIfNeeded(std::string& topicName, rclcpp::Node::SharedPtr node)
        {
            if (topicName.at(0) != '/')
            {
                std::string _namespace = node->get_namespace();

                // handle the empty namespace
                if (_namespace == "/")
                    _namespace = "";

                return _namespace + "/" + topicName;
            }
            return topicName;
        }
    } // namespace Utils
} // namespace mqtt_serialization