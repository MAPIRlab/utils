#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <mqtt_serialization/TransformJSON.hpp>

using KeyValue = diagnostic_msgs::msg::KeyValue;
using TFMessage = tf2_msgs::msg::TFMessage;

class MqttTfSync : public rclcpp::Node
{
public:
    MqttTfSync() : Node("MQTT_TF"), tf_broadcaster(*this)
    {
        mqttSub = create_subscription<KeyValue>("/mqtt2ros", 20, std::bind(&MqttTfSync::mqttCB, this, std::placeholders::_1));
        tfSub = create_subscription<TFMessage>("/tf", 20, std::bind(&MqttTfSync::tfCB, this, std::placeholders::_1));

        mqttPub = create_publisher<KeyValue>("/ros2mqtt", 20);

        framesToSend = declare_parameter<std::vector<std::string>>("framesToInclude", std::vector<std::string>{});
    }

private:
    tf2_ros::TransformBroadcaster tf_broadcaster;
    rclcpp::Subscription<KeyValue>::SharedPtr mqttSub;
    rclcpp::Subscription<TFMessage>::SharedPtr tfSub;

    rclcpp::Publisher<KeyValue>::SharedPtr mqttPub;
    std::vector<std::string> framesToSend; //if empty, sends everything

    void mqttCB(KeyValue::ConstSharedPtr msg)
    {
        if(msg->key != "/tf")
            return;
        
        nlohmann::json jsonArray(msg->value);
        for(const nlohmann::json& json : jsonArray)
        {
            geometry_msgs::msg::TransformStamped tf = mqtt_serialization::transform_from_json(json);
            //! This could cause problems, but so could keeping the original stamp if the clocks are not synchronized
            //tf.header.stamp = now(); 
            
            tf_broadcaster.sendTransform(tf);
        }
    }

    void tfCB(TFMessage::ConstSharedPtr msg)
    {
        KeyValue mqttMsg;
        mqttMsg.key = "/tf";
        nlohmann::json json;
        for(const auto& tf : msg->transforms)
        {
            if(framesToSend.empty() || contains(framesToSend, tf.child_frame_id))
                json.push_back(mqtt_serialization::transform_to_json(tf));
        }
        mqttMsg.value = json.dump();
        mqttPub->publish(mqttMsg);
    }

    template <typename Collection, typename Value>
    bool contains(const Collection& collection, const Value& value)
    {
        return std::find(collection.begin(), collection.end(), value) != collection.end();
    }
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MqttTfSync>();
    rclcpp::spin(node);
}