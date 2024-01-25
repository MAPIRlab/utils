# Installation

    sudo apt-add-repository ppa:mosquitto-dev/mosquitto-ppa

    sudo apt-get update
    
    sudo apt-get install libmosquitto1 libmosquitto-dev libmosquittopp1 libmosquittopp-dev

## Description

* MQTT_bridge:
    * Subscribes to rostopic `/ros2mqtt`, type `diagnostic_msgs::KeyValue`, which contains a "key" (the ros topic, as a string) and a "value" (a message of any arbitrary type, serialized as a string).
        * Upon receiving any messages through `/ros2mqtt`, the bridge will send those messages through MQTT to any devices that are subscribed to the corresponding topic. 
        
    * Subscribes to a list of MQTT topics/channels (see param MQTT_topic_subscribe), transform their content to `diagnostic_msgs::KeyValue`, and publish them over ROS in the topic `/mqtt2ros`. 
        * On receiving a message over MQTT, it converts the message to a key-value string pair (`diagnostic_msgs::KeyValue`) and publishes to `/mqtt2ros`.
    
    * note: The MQTT topic names can have the prefix "\[MQTT_namespace\]" (parameter of the bridge) automatically added, following the ROS topic name convention, that is:
        ```
        key=/my_topic -->  /my_topic
        key=my_topic -->  /MQTT_namespace/my_topic
        ```
        This applies both to topics published by the MQTT bridge and to topics it subscribes to.