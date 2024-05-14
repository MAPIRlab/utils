#pragma once
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <rclcpp/logging.hpp>
#include "json.hpp"

namespace mqtt_serialization
{
    inline nlohmann::json navSat_to_json(const sensor_msgs::msg::NavSatFix& navSat)
    {
        nlohmann::json json;

        json["header"]["frame_id"] = navSat.header.frame_id;
        json["header"]["stamp"]["sec"] = navSat.header.stamp.sec;
        json["header"]["stamp"]["nanosec"] = navSat.header.stamp.nanosec;

        json["status"]["status"] = navSat.status.status;
        json["status"]["service"] = navSat.status.service;
        json["latitude"] = navSat.latitude;
        json["longitude"] = navSat.longitude;
        json["altitude"] = navSat.altitude;
        
        json["position_covariance_type"] = navSat.position_covariance_type;
        json["position_covariance"] = navSat.position_covariance;

        return json;
    }

    inline sensor_msgs::msg::NavSatFix navSat_from_json(const nlohmann::json& json)
    {
        sensor_msgs::msg::NavSatFix navSat;
        try
        {
            navSat.header.frame_id = json["header"]["frame_id"].get<std::string>();
            navSat.header.stamp.sec = json["header"]["stamp"]["sec"].get<int32_t>();
            navSat.header.stamp.nanosec = json["header"]["stamp"]["nanosec"].get<int32_t>();

            navSat.status.status = json["status"]["status"].get<int8_t>();
            navSat.status.service = json["status"]["service"].get<uint16_t>();
            navSat.latitude = json["latitude"].get<double>();
            navSat.longitude = json["longitude"].get<double>();
            navSat.altitude = json["altitude"].get<double>();

            navSat.position_covariance_type = json["position_covariance_type"].get<uint8_t>();
            navSat.position_covariance = json["position_covariance"].get<std::array<double, 9>>();
        }
        catch (std::exception& e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("JSON_PARSER"), "Caught exception %s\n when trying to parse json-enconded pose: %s", e.what(), json.dump().c_str());
        }
        return navSat;
    }

}