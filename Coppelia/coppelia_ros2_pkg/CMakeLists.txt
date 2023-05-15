cmake_minimum_required(VERSION 3.5)
project(coppelia_ros2_pkg)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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

ament_target_dependencies(coppelia_simulator rclcpp)

# Declare and install executables
install(TARGETS
  coppelia_simulator
  DESTINATION lib/${PROJECT_NAME}
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(coppelia_simulator "${cpp_typesupport_target}")

# Install launch files.
install(DIRECTORY
  launch
  scenes
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
