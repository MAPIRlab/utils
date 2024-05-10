/** ****************************************************************************************
 *  GPS to local Cartesian coordinates
 *
 * Maintainer: Javier G. Monroy
 * MAPIR group: https://mapir.isa.uma.es/
 ******************************************************************************************** */

#include <gps2cartesian/gps2cartesian.hpp>
#include <gps2cartesian/navsat_conversions.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <utility>

using namespace std;
using std::placeholders::_1;

// --------------------------------------------
// CptuTrack
//---------------------------------------------

Cgps2pose::Cgps2pose() : Node("Cgps2pose")
{
    RCLCPP_INFO(this->get_logger(), "Loading Params....");
    // Read Parameters
    //----------------
    ignore_altitude_ = declare_parameter<bool>("ignore_altitude", true);
    use_cartesian_UTM_ = declare_parameter<bool>("use_cartesian_UTM", true);
    set_map_frame_latlong_manually_ = declare_parameter<bool>("set_map_frame_latlong_manually", true);
    std::string gps1_topic = declare_parameter<std::string>("gps1_topic", "methane/deluo/fix");
    std::string gps2_topic = declare_parameter<std::string>("gps2_topic", "methane/emlid/fix");
    map_frame_set_ = false;

    // Set map_frame manually?
    if (set_map_frame_latlong_manually_)
    {
        float map_latitude = declare_parameter<float>("map_latitude", 36.715839);
        float map_longitude = declare_parameter<float>("map_longitude", -4.478426);
        float map_altitude = declare_parameter<float>("map_altitude", 0.0);
        
        std::shared_ptr<sensor_msgs::msg::NavSatFix> latlong_for_map = std::make_shared<sensor_msgs::msg::NavSatFix>();        
        latlong_for_map->latitude = map_latitude;
        latlong_for_map->longitude = map_longitude;
        if (ignore_altitude_)
            latlong_for_map->altitude = 0.0;
        else
            latlong_for_map->altitude = map_altitude;
        
        //RCLCPP_INFO(this->get_logger(), "Setting map_frame at Lat(%f) and Long(%f)...",latlong_for_map->latitude,latlong_for_map->longitude);
        setMapFrame(latlong_for_map);
    }

    // Subscribers
    gps1_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(gps1_topic, 1, std::bind(&Cgps2pose::gpsFixCallback, this, _1));
    gps2_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(gps2_topic, 1, std::bind(&Cgps2pose::gpsFixCallback, this, _1));
    
    // Initialize the tf2 broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(this->get_logger(), "Ready for Tracking....");
}


Cgps2pose::~Cgps2pose()
{    
    RCLCPP_INFO(this->get_logger(), "See you later, aligator!");
}


void Cgps2pose::gpsFixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    // Check GPS frame_id
    if (msg->header.frame_id.empty()) 
    {
        RCLCPP_ERROR(this->get_logger(), "NavSatFix message has empty frame_id. Doing nothing!");
        return;
    }

    // Make sure the GPS data is usable
    bool good_gps =
        (msg->status.status != sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX &&
        !std::isnan(msg->altitude) && !std::isnan(msg->latitude) &&
        !std::isnan(msg->longitude));

    if (ignore_altitude_)
        msg->altitude = 0.0;

    if (good_gps) 
    {        
        // Is the "Map frame" already set in Cartesians
        if (!map_frame_set_) 
        {
            // Set map_frame in the Cartesian ref_system at the current GPS location
            setMapFrame(msg);
        }

        // Get GPS in Cartesian Coordinates
        double altitude = msg->altitude;
        double longitude = msg->longitude;
        double latitude = msg->latitude;
        // output data
        double cartesian_x {};
        double cartesian_y {};
        double cartesian_z {};

        // Transform Geodesic to Cartesian (either local or UTM) 
        if (!use_cartesian_UTM_) {
            // unsing Lat0 and Lon0 as the reference system
            gps_local_cartesian_.Forward(
                latitude,
                longitude,
                altitude,
                cartesian_x,
                cartesian_y,
                cartesian_z);
        } else {
            // UTM ref system of current UTM_cell
            std::string utm_zone_tmp;
            navsat_conversions::LLtoUTM(
                latitude,
                longitude,
                cartesian_y,
                cartesian_x,
                utm_zone_tmp);
        }        

        // set as TF2
        tf2::Transform cartesian_to_gps_tf;
        cartesian_to_gps_tf.setOrigin(tf2::Vector3(cartesian_x, cartesian_y, altitude));
        cartesian_to_gps_tf.setRotation(tf2::Quaternion::getIdentity());

        // Get the GPS in the map_frame as tf2
        tf2::Transform map_to_gps_tf_;
        map_to_gps_tf_.mult(cartesian_to_map_tf_.inverse(), cartesian_to_gps_tf);
        map_to_gps_tf_.setRotation(tf2::Quaternion::getIdentity());
        
        
        // publish as tf2 msg
        geometry_msgs::msg::TransformStamped t;
        tf2::convert(map_to_gps_tf_,t.transform);
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "map";
        t.child_frame_id = msg->header.frame_id;  // the one in the GPS msg
        tf_broadcaster_->sendTransform(t);

        // report
        RCLCPP_INFO(
            this->get_logger(), "GPS (latitude, longitude, altitude) = (%f, %f, %xf)",
            msg->latitude, msg->longitude, msg->altitude);
        RCLCPP_INFO(
            this->get_logger(), "GPS in map_frame (x,y,z)=(%0.2f, %0.2f, %0.2f)",
            map_to_gps_tf_.getOrigin().getX(),map_to_gps_tf_.getOrigin().getY(),map_to_gps_tf_.getOrigin().getZ());
    }
    else
    {
         RCLCPP_INFO(this->get_logger(), "GPS ignored as its not accurate (no fix).");
    }
}


// Sets the incoming GPS as the map_frame in the cartesian ref system
void Cgps2pose::setMapFrame(const sensor_msgs::msg::NavSatFix::SharedPtr & msg)
{
    RCLCPP_INFO(this->get_logger(), "Setting Map frame in Cartesians....");
    double cartesian_x {};
    double cartesian_y {};
    double cartesian_z {};
    
    // Coordinates in Cartesian
    if (!use_cartesian_UTM_) 
    {
        const double hae_altitude {};
        gps_local_cartesian_.Reset(msg->latitude, msg->longitude, hae_altitude);
        gps_local_cartesian_.Forward(
            msg->latitude,
            msg->longitude,
            msg->altitude,
            cartesian_x,
            cartesian_y,
            cartesian_z);

        // UTM meridian convergence is not meaningful when using local cartesian, so set it to 0.0
        utm_meridian_convergence_ = 0.0;
    } 
    // Use the UTM ref system, not a local one.
    else {
        navsat_conversions::LLtoUTM(
            msg->latitude,
            msg->longitude,
            cartesian_y,
            cartesian_x,
            utm_zone_,
            utm_meridian_convergence_);
        utm_meridian_convergence_ *= navsat_conversions::RADIANS_PER_DEGREE;
    }

    // report "map_fram" cartesian coordinates "DATUM"
    RCLCPP_INFO(
        this->get_logger(), "Datum (latitude, longitude, altitude) is (%0.2f, %0.2f, %0.2f)",
        msg->latitude, msg->longitude, msg->altitude);
    RCLCPP_INFO(
        this->get_logger(), "Datum %s coordinate is (%s, %0.2f, %0.2f)",
        ((!use_cartesian_UTM_) ? "Local Cartesian" : "UTM"), utm_zone_.c_str(), cartesian_x,
        cartesian_y);

    // Store as tf
    cartesian_to_map_tf_.setOrigin(tf2::Vector3(cartesian_x, cartesian_y, msg->altitude));
    cartesian_to_map_tf_.setRotation(tf2::Quaternion::getIdentity());
    map_frame_set_ = true;
}


//-----------------------------------------------------------------------------------
//                                   MAIN
//-----------------------------------------------------------------------------------
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Cgps2pose>();

    // Main Loop
    //----------
    RCLCPP_INFO(node->get_logger(), "Cgps2pose ready for operation...Looping");
    rclcpp::spin(node);

    return (0);
}
