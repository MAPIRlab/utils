#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/nav_sat_fix.hpp>

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("gps_fake_publisher")
    {
        // Read params
        gps_latitude_ = declare_parameter<float>("gps_latitude", 36.715839);
        gps_longitude_ = declare_parameter<float>("gps_longitude", -4.478426);
        gps_altitude_ = declare_parameter<float>("gps_altitude", 0.0);
        gps_frame_id_ = declare_parameter<std::string>("gps_frame_id","fake_gps");
        gps_topic_pub_ = declare_parameter<std::string>("gps_topic_pub","fake_gps");

        // Set publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(gps_topic_pub_, 1);

        // Set timer
        timer_ = this->create_wall_timer(
        100ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto msg = sensor_msgs::msg::NavSatFix();
      msg.header.stamp = this->get_clock()->now();
      msg.header.frame_id = gps_frame_id_;
      msg.status.status = 1;
      msg.latitude = gps_latitude_;
      msg.longitude = gps_longitude_;
      msg.altitude = gps_altitude_;
      RCLCPP_INFO(this->get_logger(), "Publishing Fake GPS (Lat, Long, Alt) = (%f,%f,%f)", msg.latitude, msg.longitude, msg.altitude);
      publisher_->publish(msg);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;
    float gps_latitude_, gps_longitude_, gps_altitude_;
    std::string gps_frame_id_, gps_topic_pub_;
};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}