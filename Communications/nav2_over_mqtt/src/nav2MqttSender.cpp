#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <mqtt_serialization/PoseJSON.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <spdlog/fmt/fmt.h>
#include <spdlog/spdlog.h>

using NavToPose = nav2_msgs::action::NavigateToPose;
using KeyValue = diagnostic_msgs::msg::KeyValue;

class Nav2MQTT : public rclcpp::Node
{
public:
    rclcpp_action::Server<NavToPose>::SharedPtr server;
    rclcpp::Publisher<KeyValue>::SharedPtr mqttPub;
    rclcpp::Subscription<KeyValue>::SharedPtr mqttSub;
    std::string goalTopic, resultTopic;

    std::shared_ptr<rclcpp_action::ServerGoalHandle<NavToPose>> m_activeGoalHandle;

    Nav2MQTT() : Node("Nav2MQTT")
    {
        using namespace std::placeholders;
        // server name ("topic") uses the namespace of the node
        server = rclcpp_action::create_server<NavToPose>(this, "nav2MQTT",
            std::bind(&Nav2MQTT::handle_goal, this, _1, _2),
            std::bind(&Nav2MQTT::handle_cancel, this, _1),
            std::bind(&Nav2MQTT::handle_accepted, this, _1));

        mqttPub = create_publisher<KeyValue>("/ros2mqtt", 1);
        mqttSub = create_subscription<KeyValue>("/mqtt2ros", 1, std::bind(&Nav2MQTT::mqtt_CB, this, std::placeholders::_1));

        goalTopic = declare_parameter<std::string>("goalTopic", "NavToPose");
        resultTopic = declare_parameter<std::string>("resultTopic", "NavigationResult");
        resultTopic = mqtt_serialization::Utils::applyNamespaceIfNeeded(resultTopic, shared_from_this());
        spdlog::info("Send result topic: {}", resultTopic);
    }

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const NavToPose::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<NavToPose>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");

        nlohmann::json json = mqtt_serialization::pose_to_json(goal_handle->get_goal()->pose);
        json["action"] = "cancel";

        KeyValue msg;
        msg.key = goalTopic;
        msg.value = json.dump();
        mqttPub->publish(msg);

        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<NavToPose>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Accepted goal");

        m_activeGoalHandle = goal_handle;
        nlohmann::json json = mqtt_serialization::pose_to_json(goal_handle->get_goal()->pose);
        json["action"] = "navigate";

        KeyValue msg;
        msg.key = goalTopic;
        msg.value = json.dump();
        mqttPub->publish(msg);
    }

    void mqtt_CB(KeyValue::SharedPtr msg)
    {
        if (msg->key == resultTopic)
        {
            nlohmann::json json(msg->value);
            int code = json["action_result_code"].get<int>();

            auto result = std::make_shared<nav2_msgs::action::NavigateToPose_Result>();
            if (code == (int)rclcpp_action::ResultCode::SUCCEEDED)
            {
                RCLCPP_INFO(get_logger(), "Goal completed");
                m_activeGoalHandle->succeed(result);
            }
            else
            {
                RCLCPP_WARN(get_logger(), "Goal result was not \"success\". Code: %d . See rclcpp_action::ResultCode for human-readable meaning", code);
                m_activeGoalHandle->abort(result);
            }
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Nav2MQTT>();

    rclcpp::spin(node);
}