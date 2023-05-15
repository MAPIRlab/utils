# BATTERY_MANAGER

A python node to manage the robot battery.
It is in charge of keeping a log of the battery data (voltage, intensity, etc.), as well as filtering those values in order to publish a more convenient battery topic.
Also, a very important duty is to ensure a correct level of the battery.
By default this node is configured to *warn* the user when the battery level is getting low, and also to interrupt the normal execution when a *critical* level is detected, commanding a docking action.

**Main Tasks**:

* Monitor the battery levels. Warning the user or Commanding Docking when necessary.
* Filter (low pass) the raw battery msgs and publish a filtered battery topic.
* Log battery data to file

**Important:** This node depends on the *Task_Manager* node to alert, talk and command the robot to perform docking. To do so, it makes use of the *add_new_task* service of the *Task_Manager* node.

## Parameters

* **input_battery_topic**: Name of the topic where the raw battery msgs are published (sensor_msgs/BatteryState)  

* **output_battery_topic**:  Name of the topic to publish the filtered battery msgs (sensor_msgs/BatteryState)  
  
* **filter_lenght_samples**: Lenght (in samples) of the averaging window to filter the raw battery msgs  
  
* **publish_rate**: Rate in Hz to publish the filtered battery msgs  

* **create_log**: Boolean to set if a log file should be created  
  
* **log_directory_path**: file-path where to create the logs (default is "$(find battery_manager)/logs")  

* **log_interval_sec**: To avoid continuous access to HDD, save to file each X seconds.

* **low_battery_voltage**: Voltage threshold (Volts) to warn the user about a *low* battery (it commands a SAY task)  

* **critial_battery_voltage**: Critical voltage (Volts) that once reached will command a Docking action (add_new_task->recharge_till_full).  
  
* **docking_pose**: Pose in the map where the robot should navigate to start the docking action (a pose close to the Docking station)
