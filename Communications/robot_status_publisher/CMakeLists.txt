cmake_minimum_required(VERSION 3.8)
project(robot_status_publisher)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mqtt_serialization REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)

include_directories(include)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(robot_status_publisher_node src/robot_status_publisher_node.cpp)
ament_target_dependencies(robot_status_publisher_node 
  rclcpp 
  geometry_msgs 
  sensor_msgs 
  diagnostic_msgs 
  tf2_ros 
  tf2 
  std_msgs
  mqtt_serialization 
  Boost)

target_link_libraries(robot_status_publisher_node
   ${Boost_LIBRARIES}
)

install(TARGETS
  robot_status_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
