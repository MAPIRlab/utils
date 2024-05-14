#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <mqtt_serialization/Utils.hpp>
#include <mqtt_serialization/NavSatJSON.hpp>

using NavSatFix = sensor_msgs::msg::NavSatFix;
using KeyValue = diagnostic_msgs::msg::KeyValue;

class Receiver : public rclcpp::Node
{
public:
    Receiver() : Node("NavSatMQTTReceiver")
    {
        // the topic that the message was originally published in. Will be recreated with the same name in this machine
        std::string topic = declare_parameter<std::string>("topic", "fix");
        pub = create_publisher<NavSatFix>(topic, 5);

        mqttSub = create_subscription<KeyValue>("/mqtt2ros", 5, 
            [this, topic](KeyValue::ConstSharedPtr msg) 
            {
                if(msg->key != topic)
                    return;
                NavSatFix gps = mqtt_serialization::navSat_from_json(msg->value);
                pub->publish(gps);
            });
    }

private:
    rclcpp::Subscription<KeyValue>::SharedPtr mqttSub;
    rclcpp::Publisher<NavSatFix>::SharedPtr pub;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Receiver>();
    rclcpp::spin(node);
}