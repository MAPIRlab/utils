/** ****************************************************************************************
 *  Tracking of Aruco tag with a pan-tilt unit
 *
 * Maintainer: Javier G. Monroy
 * MAPIR group: https://mapir.isa.uma.es/
 ******************************************************************************************** */

#ifndef CGPS2POSE_HPP_
#define CGPS2POSE_HPP_

#include "rclcpp/rclcpp.hpp" 
#include <Eigen/Dense>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <memory>
#include <string>

/*
  The TF chain assumed in this code is:
  cartesian -> map -> gps_frame_id

  - Cartesian: is the global UTM or lat0long0 NavSat ref system (after conversion from latlong msg)
  - map: is the "local" cartesian ref system, our (0,0) with x pointing East and y pointing North
  - gps_frame_id: the current position of the GPS

  By defatul GPS devides provide cartesian->gps_frame_id, that is the global position in cartesian (by default in UTM coordinates)
  We are interested in the map->gps (local position), for that we can set the /map frame to a fixed latlong, or use the initial GPS value
*/

class Cgps2pose : public rclcpp::Node
{
public:
    Cgps2pose();
    ~Cgps2pose();
protected:

    // GPS subscribers
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps1_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps2_sub_;

    // Callbacks
    void gpsFixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
        
    // map_frame in cartesians (our local ref system)
    bool map_frame_set_;
    bool set_map_frame_latlong_manually_;
    tf2::Transform cartesian_to_map_tf_;
    void setMapFrame(const sensor_msgs::msg::NavSatFix::SharedPtr & msg);

    // Local Cartesian Map (or UTM)
    // The origin of local cartesian coordinate system is at lat=lat0, lon=lon0, h=h0.
    // UTM sets origin as the bottom-left corner of the UTM cell
    bool use_cartesian_UTM_; //Whether we use a Local Cartesian (tangent plane ENU) or the UTM coordinates as our cartesian    
    GeographicLib::LocalCartesian gps_local_cartesian_; //Local Cartesian projection around gps origin
    
    double utm_meridian_convergence_;
    std::string utm_zone_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    bool ignore_altitude_;
};

#endif
