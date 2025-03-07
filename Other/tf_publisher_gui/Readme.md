# TF Publisher with GUI

It does what exactly you think it does.

The GUI is rendered with [ament_imgui](https://github.com/MAPIRlab/utils/tree/ros2/Visualization), and lets you change the transformation in real time so you can use Rviz visualization to gradually adjust the parameters. Once you are happy, you can use the button to print the formatted params to the terminal.

The resulting TF is by default published at 10Hz, but you can adjust the frequency with the `freq` param. Once you have correctly adjusted the transformation params, you can also run the node without the GUI by setting the `renderGUI` ROS param to `false`.