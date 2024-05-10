#include <gps_viewer/gps_viewer_node.hpp>
#include <QApplication>
#include <thread>

int main(int argc, char** argv)
{
    QApplication app(argc, argv); //TODO check this
    app.setApplicationName("ROS2-GPSViewer");

    rclcpp::init(argc, argv);
    auto node = std::make_shared<GPSViewerNode>("gps_viewer");

    rclcpp::Rate rate(60);
    while(rclcpp::ok())
    {
        rclcpp::spin_some(node);
        app.processEvents();
        rate.sleep();
    }
    
    return 0;
}

GPSViewerNode::GPSViewerNode(std::string name) : Node(name)
{
    std::string gpsTopic = declare_parameter<std::string>("gpsTopic", "fix");
    gpsSub = create_subscription<sensor_msgs::msg::NavSatFix>(gpsTopic, 1, std::bind(&GPSViewerNode::gpsCallback, this, std::placeholders::_1));

    window.show();
}

void GPSViewerNode::gpsCallback(sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
{
    window.DrawCircleAt(msg->latitude, msg->longitude, 10);
}
