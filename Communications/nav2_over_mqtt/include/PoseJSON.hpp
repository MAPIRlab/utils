#pragma once
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <json.hpp>

namespace nav2MQTT 
{

    static nlohmann::json to_json(const geometry_msgs::msg::PoseStamped& pose) 
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
        
        return json;
    }

    static geometry_msgs::msg::PoseStamped from_json(const nlohmann::json& json)
    {
        geometry_msgs::msg::PoseStamped pose;

        pose.header.frame_id = json["header"]["frame_id"];

        pose.pose.position.x = json["pose"]["position"]["x"].get<double>();
        pose.pose.position.y = json["pose"]["position"]["y"].get<double>();
        pose.pose.position.z = json["pose"]["position"]["z"].get<double>();

        pose.pose.orientation.w = json["pose"]["orientation"]["w"].get<double>();
        pose.pose.orientation.x = json["pose"]["orientation"]["x"].get<double>();
        pose.pose.orientation.y = json["pose"]["orientation"]["y"].get<double>();
        pose.pose.orientation.z = json["pose"]["orientation"]["z"].get<double>();
        
        return pose;
    }

}