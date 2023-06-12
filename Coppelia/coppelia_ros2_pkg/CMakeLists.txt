cmake_minimum_required(VERSION 3.5)
project(coppelia_ros2_pkg)


set(USE_API ON)


set(CMAKE_BUILD_TYPE "None")
set(CMAKE_CXX_FLAGS "-std=c++17 -g -Og ${CMAKE_CXX_FLAGS}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# msg generation
rosidl_generate_interfaces(${PROJECT_NAME}
"msg/RangeBearing.msg"
"msg/Ultrasonic.msg"
DEPENDENCIES builtin_interfaces std_msgs
)

# Add Executables
add_executable(coppelia_simulator src/run_coppelia_simulator.cpp)

if(USE_API)
    find_package(coppelia_remote_api REQUIRED)
    target_link_libraries(coppelia_simulator zmq)
    target_compile_definitions(coppelia_simulator PRIVATE -DUSE_API)
endif()

ament_target_dependencies(coppelia_simulator rclcpp coppelia_remote_api)



rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(coppelia_simulator "${cpp_typesupport_target}")




# Declare and install executables
install(TARGETS
  coppelia_simulator
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  scenes
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
