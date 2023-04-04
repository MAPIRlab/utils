#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <vector>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <boost/bind.hpp>

#include "mqtt_bridge/CMQTTmosquitto2.hpp"

using namespace std;

/**
 This Pkg provides a bridge between ROS and the MQTT mosquitto architecture:
 - Mosquitto MQTT is a machine-to-machine (M2M)/"Internet of Things" connectivity protocol, designed as an extremely lightweight publish/subscribe messaging transport.
 - ROS controls the robots locally and is based on a Topic pub/sub system

 The bridge:
      Passes orders from Mosquitto (on the MQTT topic "MQTT_topicName/#") to the ROS Robot Operating System (ROS topic: mqtt2ros)
      Passes commands from ROS topic (ROS topic: ros2mqtt) to MQTT mosquitto MQTT_topicName/#

	      MQTT                       ROS
	MQTT_topicName/#       -->     mqtt2ros
	MQTT_topicName/#       <--     ros2mqtt

	MQTT_msg   to  ROS_msg conversion:
	topic     <-->   key
	payload   <-->   value

 note: In MQTT all topic names will have the prefix "MQTT_topicName", that is:
        topic = MQTT_topicName/my_topic    <-->    key=my_topic
**/



// Callback function when new ROS msg is received on topic "ros2mqtt"
// Transform the ROS msg to MQTT format and publish it

char id [100];
CMQTTMosquitto *MQTTconnector= new CMQTTMosquitto(id);

//MQTTconnector 


void ros2mqtt_callback(const diagnostic_msgs::msg::KeyValue msg)
{

    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),"Sending msg to MQTT: topic=%s    value=:%s", 
              msg.key.c_str(), msg.value.c_str());
    //publish(NULL, topic_name, payload_length, data)
    MQTTconnector->on_publish(NULL, 
                                (MQTTconnector->MQTT_topicName+"/"+msg.key).c_str(), 
                                strlen(msg.value.c_str()), 
                                msg.value.c_str());
}


// MAIN
int main(int argc, char** argv)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"[mqtt_bridge] ");
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("mqtt_bridge_node");

    // Init mosquittopp lib (providing an unique ID)
    char id [100];

    sprintf(id, "UMArobot_%f", rclcpp::Clock().now().seconds());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"[mqtt_bridge] Connecting to mosquittopp with ID: %s",id);



    // Topics
    using std::placeholders::_1;

    rclcpp::Subscription<diagnostic_msgs::msg::KeyValue>::SharedPtr sub;

    sub = node->create_subscription<diagnostic_msgs::msg::KeyValue>("/ros2mqtt", 1000, std::bind(&ros2mqtt_callback,_1));
    
    MQTTconnector->ros_pub = node->create_publisher<diagnostic_msgs::msg::KeyValue>("/mqtt2ros", 100);



    //Get node parameters
    //--------------------
    MQTTconnector->broker_host = node->declare_parameter<std::string>("host", "150.214.109.137");

    MQTTconnector->broker_port = node->declare_parameter<int>("port", 8002);

    MQTTconnector->broker_username = node->declare_parameter<std::string>("username", "");

    MQTTconnector->broker_password = node->declare_parameter<std::string>("password", "");

    MQTTconnector->MQTT_topicName = node->declare_parameter<std::string>("MQTT_topicName", "MAPIR_robot"); //To allow multiple robots.

    MQTTconnector->MQTT_topics_subscribe = node->declare_parameter<std::string>("MQTT_topics_subscribe", "NavigationCommand,ClientACK,ServerACK");

    MQTTconnector->append_timestamp = node->declare_parameter<bool>("append_timestamp", false); // Allows adding a timeStamp before each msgTo allow multiple robots.


    // Connect to MQTTmosquitto broker
    //---------------------------------
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"[mqtt_bridge] Initializing mqtt_bridge at %s:%u",MQTTconnector->broker_host.c_str(),MQTTconnector->broker_port);

    if (MQTTconnector->broker_username != "")
    {
        if(MQTTconnector->username_pw_set(MQTTconnector->broker_username.c_str(), MQTTconnector->broker_password.c_str()) )
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"Error setting username and password");
    }

    // connect(username, pass, keepAlive(sec))
    int rc = MQTTconnector->connect(MQTTconnector->broker_host.c_str(), MQTTconnector->broker_port, 10);

    //On success, subscribe to MQTT topic
    //MQTTconnector.on_connect(rc);


    //-------------
    // MAIN Loop
    //-------------
    rclcpp::Rate loop_rate(50);
    while(rclcpp::ok())
    {
        //MQTT --> ROS
        int rc = MQTTconnector->loop(-1);    //Calls on_msg if any MQTT new msg is available
        MQTTconnector->checkConnection(rc);  //Re-connect if necessary
        //ROS --> MQTT
        rclcpp::spin_some(node);    //Callbacks of subscribed ROS topics        

        loop_rate.sleep();
    }
   
    return(0);
}