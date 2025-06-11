#include <filesystem>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <yaml-cpp/yaml.h>

using nav_msgs::msg::OccupancyGrid;

class MapServer : public rclcpp::Node
{
public:
    MapServer()
        : Node("MapServer")
    {}
    void Run();

private:
    void ReadMap();
    geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw);

private:
    OccupancyGrid msg;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapServer>();
    node->Run();
    return 0;
}

////////////////////////////////////////
////////////////////////////////////////

inline void MapServer::Run()
{
    ReadMap();
    std::string topic = declare_parameter<std::string>("topic", "map");
    auto pub = create_publisher<OccupancyGrid>(topic, rclcpp::QoS(1).transient_local());

    double publishFreq = declare_parameter<double>("frequency", 0.1);
    RCLCPP_INFO(get_logger(), "Publishing map on topic '%s' at %.2fHz", pub->get_topic_name(), publishFreq);
    rclcpp::Rate rate(publishFreq);
    while (rclcpp::ok())
    {
        msg.header.stamp = now();
        pub->publish(msg);
        rate.sleep();
    }
}

inline void MapServer::ReadMap()
{
    std::string mapYAMLfilepath = declare_parameter<std::string>("yaml_filename", "");
    try
    {
        YAML::Node yaml = YAML::LoadFile(mapYAMLfilepath);

        // if the image path is relative interpret it as relative to the YAML, not the working directory
        std::filesystem::path imagePath(yaml["image"].as<std::string>());
        if (imagePath.is_relative())
            imagePath = std::filesystem::path(mapYAMLfilepath).parent_path() / imagePath;

        // data
        //-----------------------
        cv::Mat mapImage = cv::imread(imagePath, cv::IMREAD_GRAYSCALE);
        size_t width = mapImage.size().width;
        size_t height = mapImage.size().height;
        msg.data.resize(width * height);

        double free_thresh = 1 - yaml["free_thresh"].as<double>(); // Unused, this is for trinary maps. We only consider cells free or occupied, not "unknown"
        double occupied_thresh = 1 - yaml["occupied_thresh"].as<double>();

        for (int y = 0; y < height; y++)
            for (int x = 0; x < width; x++)
            {
                double value = mapImage.at<uint8_t>(y, x) / 255.;
                int8_t& occupancy = msg.data.at(width * (height - y - 1) + x);
                if (value > free_thresh)
                    occupancy = 0;
                else if (value < occupied_thresh)
                    occupancy = 100;
                else
                    occupancy = -1;
            }

        // metadata
        //-------------------
        msg.header.frame_id = "map";

        msg.info.map_load_time = now();
        msg.info.resolution = yaml["resolution"].as<double>();
        auto origin_array = yaml["origin"].as<std::array<double, 3>>();
        msg.info.origin.position.x = origin_array[0];
        msg.info.origin.position.y = origin_array[1];
        msg.info.origin.orientation = createQuaternionMsgFromYaw(origin_array[2]);

        msg.info.height = height;
        msg.info.width = width;

#if 0
//TODO maybe add support for map modes and negation?
auto map_mode_node = doc["mode"];
if (!map_mode_node.IsDefined()) {
    load_parameters.mode = MapMode::Trinary;
} else {
    load_parameters.mode = map_mode_from_string(map_mode_node.as<std::string>());
}

try {
    load_parameters.negate = yaml_get_value<int>(doc, "negate");
} catch (YAML::Exception &) {
    load_parameters.negate = yaml_get_value<bool>(doc, "negate");
}
#endif
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(get_logger(), "Exception caught: '%s'", e.what());
    }
}

geometry_msgs::msg::Quaternion MapServer::createQuaternionMsgFromYaw(double yaw)
{
    return tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), yaw));
}