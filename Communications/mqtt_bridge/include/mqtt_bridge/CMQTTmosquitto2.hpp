#ifndef CMQTTMosquitto_HPP
#define CMQTTMosquitto_HPP

#include <rclcpp/rclcpp.hpp>
#include "diagnostic_msgs/msg/key_value.hpp"
#include <mosquitto.h>
#include <mosquittopp.h>
#include <inttypes.h>
#include <boost/format.hpp>
#include <cstdio>
#include <cstring>

using namespace std;

class CMQTTMosquitto : public mosqpp::mosquittopp
{
public:
    CMQTTMosquitto(const char *id) : mosquittopp(id)
    {
        mosqpp::lib_init();			// Initialize libmosquitto
        append_timestamp = false;   // init
    }

    void on_connect(int rc){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"MQTT Connected with code %u", rc);
        switch(rc)
        {
            case 0:
            {
                // Successful connect
                //Subscribe to list of MQTT topics
                bool more_topics = true;
                std::string myTopics = MQTT_topics_subscribe;
                while(more_topics)
                {
                    size_t pos = myTopics.find(",");
                    //hande absolute topic names ("/topic") vs namespace-relative ones ("topic")
                    std::string tokenInList = myTopics.substr(0, pos);
                    std::string completeTopic;

                    //remove whitespaces
                    while(tokenInList.at(0)==' ')
                        tokenInList.substr(1);


                    if(tokenInList.at(0)=='/')
                        completeTopic = tokenInList;
                    else
                        completeTopic = MQTT_namespace+"/"+tokenInList;
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"[mqtt_bridge] Subscribing to %s",completeTopic.c_str() );

                    int rc = subscribe(NULL, completeTopic.c_str(), 0);
                    if (rc)
                        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"[mqtt_bridge] Error: failed to subscribe to %s/# with error code %u", MQTT_namespace.c_str(), rc);

                    if (pos == -1)
                    {
                        //Last topic
                        more_topics = false;
                    }
                    else
                    {
                        myTopics.erase(0, pos+1);
                    }
                }

                //Subscribe to ALL (Debug)
                /*int rc = subscribe(NULL, (MQTT_topicName+"/#").c_str(), 0);
                if (rc)
                    ROS_ERROR("Error: failed to subscribe to %s/# with error code %u", MQTT_topicName.c_str(), rc);
                */

                //Set last-will
                char lastWill [100];
                sprintf(lastWill, "ByeMQTT");
                will_set("lastWill", 7, lastWill, 2, true);

                break;
            }
            case 1:
                std::cerr << "[mqtt_bridge] Connection Refused: unacceptable protocol version\n";
                break;
            case 2:
                std::cerr << "[mqtt_bridge] Connection Refused: identifier rejected\n";
                break;
            case 3:
                std::cerr << "[mqtt_bridge] Connection Refused: broker unavailable\n";
                break;
            case 4:
                std::cerr << "[mqtt_bridge] Connection Refused: bad user name or password\n";
                break;
            case 5:
                std::cerr << "[mqtt_bridge] Connection Refused: not authorised\n";
                break;
            default:
                std::cerr << "[mqtt_bridge] Connection Refused: unknown reason\n";
                break;
        }
    };
    void checkConnection(int rc){
        // Check connection with Broker
        //------------------------------    
        if(rc!=MOSQ_ERR_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"[mqtt_bridge] Connection lost with MQTT broker, error code: %u. Reconnecting...", rc);
            disconnect();
            mosqpp::lib_cleanup();
            mosqpp::lib_init();
            if (broker_username != "")
            {	
                if(username_pw_set(broker_username.c_str(), broker_password.c_str()) )
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"[mqtt_bridge] Error setting username and password");
            }        
            connect(broker_host.c_str(), broker_port, 30);

            //On success, subscribe to MQTT topic
            //on_connect(rc);
        }
    };
    void on_message(const struct mosquitto_message *message){
        // convert from MQTT_msg to ROS_msg and publish it over the mqtt2ros topic
        std::string msgTopic = string(message->topic);
        char * cstr = new char[message->payloadlen + 1];
        std::strcpy(cstr, (char*)message->payload );
        std::string msgLoad = string(cstr);

        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),"[mqtt_bridge] Received msg from MQTT: topic=%s   value=%s", msgTopic.c_str(), msgLoad.c_str());

        //Publish msg on ROS Topic mqtt2ros
        diagnostic_msgs::msg::KeyValue msg;
        msg.key = msgTopic;
        msg.value = msgLoad;
        //Publish on mqtt2ros
        ros_pub->publish(msg);
        delete[] cstr;
    };

    void on_publish(int *mid, const char *topic, int payloadlen, const void *payload){
        if (append_timestamp)
        {
            //Append "current_timestamp|" to all published messages, as part of the payload message
            char * cstr = new char[payloadlen + 1];
            std::strcpy(cstr, (char*)payload );
            uint64_t tt = (uint64_t)rclcpp::Clock().now().seconds() * 1000;	//timestamp in ms
            std::string myPayload = boost::str( boost::format("%lu|%s") % tt % cstr );  //So much for the simple mrpt::format -_-

            //Update vars
            payloadlen = strlen(myPayload.c_str());

            //Send message over MQTT with qos=0
            publish(mid, topic, payloadlen, myPayload.c_str(),0,false);
            delete[] cstr;
        }
        else
        {
            //Send message over MQTT with qos=0
            publish(mid, topic, payloadlen, payload, 0, false);
        }
    };

    //variables
    std::string broker_host, broker_username, broker_password, MQTT_namespace, MQTT_topics_subscribe;
    int broker_port;
    bool append_timestamp;
    rclcpp::Publisher<diagnostic_msgs::msg::KeyValue>::SharedPtr ros_pub;
};

#endif
