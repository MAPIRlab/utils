#pragma once
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "Utils.hpp"
#include "json.hpp"

namespace mqtt_serialization
{
    inline nlohmann::json transform_to_json(const geometry_msgs::msg::TransformStamped& tf)
    {
        nlohmann::json json;

        json["header"]["frame_id"] = tf.header.frame_id;
        json["header"]["stamp"]["sec"] = tf.header.stamp.sec;
        json["header"]["stamp"]["nanosec"] = tf.header.stamp.nanosec;

        json["transform"]["translation"]["x"] = tf.transform.translation.x;
        json["transform"]["translation"]["y"] = tf.transform.translation.y;
        json["transform"]["translation"]["z"] = tf.transform.translation.z;

        json["transform"]["rotation"]["w"] = tf.transform.rotation.w;
        json["transform"]["rotation"]["x"] = tf.transform.rotation.x;
        json["transform"]["rotation"]["y"] = tf.transform.rotation.y;
        json["transform"]["rotation"]["z"] = tf.transform.rotation.z;

        // aditional redundant orientation field for human-readability
        json["transform"]["rotation"]["yaw"] = Utils::getYaw(tf.transform.rotation);

        return json;
    }

    inline geometry_msgs::msg::TransformStamped transform_from_json(const nlohmann::json& json)
    {
        geometry_msgs::msg::TransformStamped tf;
        try
        {
            tf.header.frame_id = json["header"]["frame_id"].get<std::string>();
            tf.header.stamp.sec = json["header"]["stamp"]["sec"].get<int32_t>();
            tf.header.stamp.nanosec = json["header"]["stamp"]["nanosec"].get<int32_t>();

            tf.transform.translation.x = json["transform"]["translation"]["x"].get<double>();
            tf.transform.translation.y = json["transform"]["translation"]["y"].get<double>();
            tf.transform.translation.z = json["transform"]["translation"]["z"].get<double>();

            tf.transform.rotation.w = json["transform"]["rotation"]["w"].get<double>();
            tf.transform.rotation.x = json["transform"]["rotation"]["x"].get<double>();
            tf.transform.rotation.y = json["transform"]["rotation"]["y"].get<double>();
            tf.transform.rotation.z = json["transform"]["rotation"]["z"].get<double>();
        }
        catch (std::exception& e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("JSON_PARSER"), "Caught exception %s\n when trying to parse json-enconded transform: %s", e.what(), json.dump().c_str());
        }
        return tf;
    }

}