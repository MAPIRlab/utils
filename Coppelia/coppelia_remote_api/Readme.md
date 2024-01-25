# Remote API

You might have noticed this package does not contain any code. It's actually just an ament wrapper for the [Coppelia ZMQ Remote API](https://www.coppeliarobotics.com/helpFiles/en/zmqRemoteApiOverview.htm), the source code of which is distributed with Coppelia itself. The point is to make it so all you need to do to use the remote API in a ROS package is to add the following to your `CMakeLists.txt`:

```
find_package(coppelia_remote_api REQUIRED)
ament_target_dependencies([your_executable] coppelia_remote_api)
```

## External dependecies

ZeroMQ
```
sudo apt-get install libzmq3-dev
```