#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <mqtt_serialization/PoseJSON.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <spdlog/fmt/fmt.h>
#include <spdlog/spdlog.h>

using NavToPose=nav2_msgs::action::NavigateToPose;
using KeyValue=diagnostic_msgs::msg::KeyValue;

class Mqtt2Nav2 : public  rclcpp::Node
{
public:
    NavToPose::Goal goal;
    bool pending = false;
    rclcpp_action::Client<NavToPose>::SharedPtr nav2client;
    rclcpp::Subscription<KeyValue>::SharedPtr mqttSub;
    rclcpp::Publisher<KeyValue>::SharedPtr mqttPub;

    std::string receive_goal_topic_mqtt;
    std::string send_result_topic_mqtt;

    Mqtt2Nav2() : Node("MQTT_to_nav2")
    {
        nav2client = rclcpp_action::create_client<NavToPose>(this, "navigate_to_pose");
        mqttSub = create_subscription<KeyValue>("/mqtt2ros", 1, std::bind(&Mqtt2Nav2::MQTT_callback, this, std::placeholders::_1));
        mqttPub = create_publisher<KeyValue>("/ros2mqtt", 1);

        receive_goal_topic_mqtt = declare_parameter<std::string>("receive_goal_topic_mqtt", "NavToPose");
        receive_goal_topic_mqtt = applyNamespaceIfNeeded(receive_goal_topic_mqtt);
        spdlog::info("Receive goal topic: {}", receive_goal_topic_mqtt);
        send_result_topic_mqtt = declare_parameter<std::string>("send_result_topic_mqtt", "NavigationResult");
        spdlog::info("Send result topic: {}", send_result_topic_mqtt);
    }

    void MQTT_callback(KeyValue::SharedPtr msg)
    {
        if(msg->key == receive_goal_topic_mqtt)
        {
            nlohmann::json json = nlohmann::json::parse(msg->value);

            //action [navigate, cancel]
            std::string action = json["action"].get<std::string>();
            if(action == "navigate")
            {
                goal.pose = mqtt_serialization::pose_from_json(json);
                goal.pose.header.stamp = now();

                rclcpp_action::Client<NavToPose>::SendGoalOptions options;
                options.result_callback = std::bind(&Mqtt2Nav2::nav2GoalDone, this, std::placeholders::_1);
                nav2client->async_send_goal(goal);
                spdlog::info("Sending goal to nav2");
            }
            else if (action == "cancel")
            {
                nav2client->async_cancel_all_goals();
                spdlog::info("Cancelling goal");
            }
            
            else
                spdlog::error("Unknown action requested: {}. Must be one of[\"navigate\", \"cancel\"]", action);
        }
    }

    void nav2GoalDone(const rclcpp_action::ClientGoalHandle<NavToPose>::WrappedResult &result)
    {
        nlohmann::json json;
        json["action_result_code"] = result.code;
        spdlog::info("Goal complete, sending result");

        KeyValue msg;
        msg.key = send_result_topic_mqtt;
        msg.value = json.dump();
        mqttPub->publish(msg);
    }

    std::string applyNamespaceIfNeeded(std::string& topicName)
    {
        if(topicName.at(0) != '/')
        {
            std::string _namespace = get_namespace();

            //handle the empty namespace
            if(_namespace == "/")
                _namespace = "";

            return fmt::format("{}/{}", _namespace, topicName);
        }
        return topicName;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Mqtt2Nav2>();

    rclcpp::spin(node);
    return 0;
}