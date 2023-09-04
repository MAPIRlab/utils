#pragma once
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "json.hpp"

namespace mqtt_serialization
{
    namespace Utils
    {
        static double getYaw(const geometry_msgs::msg::Quaternion& quat)
        {
            tf2::Quaternion tfquat;
            tf2::fromMsg(quat, tfquat);

            tf2::Matrix3x3 m(tfquat);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            return yaw;
        }
    }

    static nlohmann::json pose_to_json(const geometry_msgs::msg::PoseStamped& pose)
    {
        nlohmann::json json;

        json["header"]["frame_id"] = pose.header.frame_id;

        json["pose"]["position"]["x"] = pose.pose.position.x;
        json["pose"]["position"]["y"] = pose.pose.position.y;
        json["pose"]["position"]["z"] = pose.pose.position.z;

        json["pose"]["orientation"]["w"] = pose.pose.orientation.w;
        json["pose"]["orientation"]["x"] = pose.pose.orientation.x;
        json["pose"]["orientation"]["y"] = pose.pose.orientation.y;
        json["pose"]["orientation"]["z"] = pose.pose.orientation.z;

        // aditional redundant orientation field for human-readability
        json["pose"]["orientation"]["yaw"] = Utils::getYaw(pose.pose.orientation);

        return json;
    }

    static geometry_msgs::msg::PoseStamped pose_from_json(const nlohmann::json& json)
    {
        geometry_msgs::msg::PoseStamped pose;
        try
        {
            pose.header.frame_id = json["header"]["frame_id"].get<std::string>();

            pose.pose.position.x = json["pose"]["position"]["x"].get<double>();
            pose.pose.position.y = json["pose"]["position"]["y"].get<double>();
            pose.pose.position.z = json["pose"]["position"]["z"].get<double>();

            pose.pose.orientation.w = json["pose"]["orientation"]["w"].get<double>();
            pose.pose.orientation.x = json["pose"]["orientation"]["x"].get<double>();
            pose.pose.orientation.y = json["pose"]["orientation"]["y"].get<double>();
            pose.pose.orientation.z = json["pose"]["orientation"]["z"].get<double>();
        }
        catch (std::exception& e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("JSON_PARSER"), "Caught exception %s\n when trying to parse json-enconded pose: %s", e.what(), json.dump().c_str());
        }
        return pose;
    }

}