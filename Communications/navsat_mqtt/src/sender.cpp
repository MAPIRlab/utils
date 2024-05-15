#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <mqtt_serialization/Utils.hpp>
#include <mqtt_serialization/NavSatJSON.hpp>

using NavSatFix = sensor_msgs::msg::NavSatFix;
using KeyValue = diagnostic_msgs::msg::KeyValue;

class Sender : public rclcpp::Node
{
public:
    Sender() : Node("NavSatMQTTSender")
    {}

    void Initialize()
    {
        mqttPub = create_publisher<KeyValue>("/ros2mqtt", 5);

        std::string topic = declare_parameter<std::string>("topic", "fix");
        std::string completeTopicName = mqtt_serialization::Utils::applyNamespaceIfNeeded(topic, shared_from_this());
        sub = create_subscription<NavSatFix>(topic, 5, [this, completeTopicName](NavSatFix::ConstSharedPtr gps) {
            KeyValue keyValue;
            keyValue.key = completeTopicName;
            keyValue.value = mqtt_serialization::navSat_to_json(*gps).dump();
            mqttPub->publish(keyValue);
        });
    }

private:
    rclcpp::Subscription<NavSatFix>::SharedPtr sub;
    rclcpp::Publisher<KeyValue>::SharedPtr mqttPub;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Sender>();
    node->Initialize();
    rclcpp::spin(node);
}