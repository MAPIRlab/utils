# Instalation

    sudo apt-add-repository ppa:mosquitto-dev/mosquitto-ppa

    sudo apt-get update
    
    sudo apt-get install libmosquitto1 libmosquitto-dev libmosquittopp1 libmosquittopp-dev

## Description

* MQTT_bridge:
    * Subscribes to rostopic /ros2mqtt, type diagnostic_msgs::KeyValue, which contains a "key" (string) and a "value" (string).
        * On received msg over /ros2mqtt, it publishes a message to MQTT over mosquitto, under the topic MQTT_topicName/key, with the content described in "value". The parameter MQTT_topicName can be configured in the launchfile to avoid namespace colission. Allowing multiple robots simulatenously to operate on the same network.
        
    * Subscribes to a list of MQTT topics/channels (see param MQTT_topic_subscribe), transform their content to diagnostic_msgs::KeyValue, and publish them over ROS in the topic /mqtt2ros. 
        * On received message over MQTT, it converts the message to a key-value string pair (diagnostic_msgs::KeyValue) and publishes to `/mqtt2ros`, which can then be processed by other nodes such as `mapir-iot-api`.