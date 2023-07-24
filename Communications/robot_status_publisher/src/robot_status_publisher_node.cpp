/** ****************************************************************************************
*  This node implements a simple publisher of the current robot status
*  Designed to serve as a monitor of the robot status.
*
* Maintainer: Javier G. Monroy
* MAPIR group: https://mapir.isa.uma.es/
******************************************************************************************** */

#include "robot_status_publisher/robot_status_publisher_node.h"
#include <sstream>
#include <stdlib.h>     /* atoi */
#include <vector>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2/exceptions.h"
#include "tf2/utils.h"

using namespace std;
using json = nlohmann::json;
using std::placeholders::_1;

// --------------------------------------------
// CrobotStatus
//---------------------------------------------

CrobotStatus::CrobotStatus() : Node("CrobotStatus")
{
    // Read Parameters
    //----------------
    output_topic = this->declare_parameter<std::string>("output_topic", "/ros2mqtt");
    statusRate = this->declare_parameter<double>("statusRate", 1.0);
    map_frame = this->declare_parameter<std::string>("map_frame", "map");
    base_frame = this->declare_parameter<std::string>("base_frame", "base_link");
    battery_topic = this->declare_parameter<std::string>("battery_topic", "/battery_filtered");
    
    // Subscribers
    //-------------    
    battery_sub = this->create_subscription<sensor_msgs::msg::BatteryState>(battery_topic, 1, std::bind(&CrobotStatus::batteryCallBack, this, _1));
    task_sub = this->create_subscription<std_msgs::msg::String>("/bt_manager/current_task", 1, std::bind(&CrobotStatus::runningTaskCallBack, this, _1));
    current_task.data = "NOT YET INITIALIZED";
    // For the pose we use the TF instead of reading a topic.

    // Publisher
    //-------------
    status_pub = this->create_publisher<diagnostic_msgs::msg::KeyValue>(output_topic, 5);

    //Init battery
    current_battery.voltage = 0.0;      //Voltage in Volts (Mandatory)
    current_battery.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
    current_battery.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
    current_battery.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
    current_battery.present = true;
    
}


CrobotStatus::~CrobotStatus()
{
}

//! sendStatus report the robot status via a JSON string.
//! It contains: robot localization, topological localization, battery_status, plus extra sensor readings
void CrobotStatus::sendStatus()
{
    // Send robot STATUS (json)
        //-------------------------
        /* FORMAT
            {
            "time" : {"temporality" : "timestamp", "t" : <timestamp>},
            "data" : { "pose" : "{
                                    "position":
                                    {
                                        "x":<x>,
                                        "y":<y>,
                                        "z":<z>
                                    },
                                    "orientation":
                                    {
                                        "x":<x>,
                                        "y":<y>,
                                        "z":<z>,
                                        "w":<w>,
                                        "yaw":<yaw (radians)>,
                                    },
                                }
                       "topological_place" : "<LOCATION_CODE>",
                       "battery" : {"voltage" : "<float>", "percentage" : "<float[0-1]>", "power_supply_status" : "1=charging, 2-3=discharging, 4 =full"},
                       "current_task":"IDLE, Talk, goto_pose,..."
                     }
            }
        */
    try
    {
        // compose json msg to send
        json j;
        j["time"]["temporality"] = "timestamp";
        j["time"]["t"] = std::to_string(rclcpp::Clock().now().seconds());


        // 1. Add robot pose in the map_frame
        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer->lookupTransform(map_frame, base_frame, tf2::TimePointZero,100ms);
            geometry_msgs::msg::PoseStamped pose;
            pose.header = transform.header;
            pose.pose.position.x = transform.transform.translation.x;
            pose.pose.position.y = transform.transform.translation.y;
            pose.pose.position.z = transform.transform.translation.z;
            pose.pose.orientation = transform.transform.rotation;
            // format as JSON
            j["data"]["pose"] = mqtt_serialization::pose_to_json(pose);
            
        } catch(tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(),"Exception while reading the robot pose -------> %s", ex.what());
            j["data"]["pose"] = "Unavailable";
        }


        // 2. topological localization (location_name)
        j["data"]["topological_place"] = "Unknown";


        //3. Battery Status
        //UNKNOWN = 0, CHARGING = 1, DISCHARGING = 2, NOT_CHARGING = 3, FULL = 4
        j["data"]["battery"]["voltage"] = boost::str( boost::format("%.2f") % current_battery.voltage );
        j["data"]["battery"]["percentage"] = boost::str( boost::format("%.2f") % current_battery.percentage );  // range [0-1]
        j["data"]["battery"]["power_supply_status"] = boost::str( boost::format("%u") % (int)current_battery.power_supply_status );


        //4. Current task in execution
        j["data"]["intervention"] = current_task.data;


        // Publish
        diagnostic_msgs::msg::KeyValue msg;
        msg.key = "status";                  //MQTT-Subtopic where to publish the data
        msg.value = j.dump();
        status_pub->publish(msg);

        // Debug
        RCLCPP_INFO(this->get_logger(),"STATUS is: [%s]", msg.value.c_str());
    }
    catch(std::runtime_error& ex)
    {
        RCLCPP_ERROR(this->get_logger(),"Exception when sending STATUS: [%s]", ex.what());
    }
}


//-----------------------------------------------------------------------------------
//                                   CALLBACKS
//-----------------------------------------------------------------------------------
void CrobotStatus::batteryCallBack(const sensor_msgs::msg::BatteryState::SharedPtr new_battery)
{
    // update battery status
    current_battery = *new_battery;
}


void CrobotStatus::runningTaskCallBack(const std_msgs::msg::String::SharedPtr new_running_task)
{
    // update current task being executted
    current_task = *new_running_task;
}


//-----------------------------------------------------------------------------------
//                                   MAIN
//-----------------------------------------------------------------------------------
int main(int argc, char** argv)
{    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CrobotStatus>();
    
    // Main Loop
    //----------
    RCLCPP_INFO(node->get_logger(),"Status-Publisher ready for operation...Looping");
    rclcpp::Rate loop_rate(node->statusRate);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);        //Check for new msgs (callabacks)
        node->sendStatus();
        loop_rate.sleep();
    }
    return(0);
}
