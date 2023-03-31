# INSTALLATION OF THE CHROME EXTENSION (PLUGIN)
1. Open a Chrome web browser.
2. Go to "More tools" > "Extensions".
3. Activate the "Developer mode" tick.
4. Click on "Load unpacked extension".
5. Go to "plugins/TTS" directory and click "Open".
6. Copy the plugin ID (shown just below the `Detail` link in the extension tab) to the corresponding param in the node launch file. E.g. `<param name="script_id" value="gcfnecbmoamdbkommkbdmnjcepmadiom"/>`. The node will automatically launch the Chrome plugin when started.

# HITS
1. This ros node depends on mqtt_bridge.
2. Configure the MQTT port and Topic to subscribe in `.speed/plugings/TTS/ttsMQTT.js`
3. It is convenient to create a shortcut of the extension in the desktop.


# HOW TO USE
This node implements a service (**say_text text_to_play**) to easily reproduce text strings.  
You can test it directly from the terminal using:  
`rosservice call /speech/say_text "Exterminar"`


**Recomendation:**  
Instead of calling this service directly from your ROS pkgs, it is recommended to make use of the 
[TaskManager](https://gitlab.com/mapir/mapir-ros-sources/tree/kinetic-dev/planning/task_manager) and the "add_new_task" service, to create a task to say a text. 
This way we can priorize the tasks and have a better control of the robot. 

` rosservice call /bt_manager/add_new_task say 8 False ['"Exterminar"']`
