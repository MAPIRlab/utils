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
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>

/*
  The TF chain assumed in this code is:
  cartesian -> map -> gps

  - Cartesian: is the global UTM or lat0long0 NavSat ref system
  - map: is the "local" cartesian ref system, our (0,0)
  - gps: the current position of the GPS

  The GPS provides cartesian->gps transform (global pos)
  We are interested in the map->gps (local pos)
*/

class Cgps2pose : public rclcpp::Node
{
public:
    Cgps2pose();
    ~Cgps2pose();
protected:

    // GPS
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    void gpsFixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);    
    tf2::Transform latest_map_to_gps_tf_;
    Eigen::MatrixXd latest_gps_cartesian_covariance_;
    
    // map_frame in cartesians (our local ref system)
    bool map_frame_set_;
    tf2::Transform cartesian_to_map_tf_;
    void setMapFrame(const sensor_msgs::msg::NavSatFix::SharedPtr & msg);

    // Local Cartesian Map (or UTM)
    // The origin of local cartesian coordinate system is at lat=lat0, lon=lon0, h=h0.
    // UTM sets origin as the bottom-left corner of the cell
    bool use_lat0long0_cartesian_; //Whether we use a Local Cartesian (tangent plane ENU) or the UTM coordinates as our cartesian
    GeographicLib::LocalCartesian gps_local_cartesian_; //Local Cartesian projection around gps origin
    
    double utm_meridian_convergence_;
    std::string utm_zone_;
};

#endif
