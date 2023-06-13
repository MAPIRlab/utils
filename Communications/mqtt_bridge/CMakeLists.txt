cmake_minimum_required(VERSION 3.8)
project(mqtt_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)

# Find the native libmosquitto includes and libraries
find_path(MOSQUITTO_INCLUDE_DIR mosquitto.h mosquittopp.h)
find_library(MOSQUITTO_LIBRARY NAMES mosquitto)
find_library(MOSQUITTOpp_LIBRARY NAMES mosquittopp)
set(MOSQUITTO_LIBRARIES ${MOSQUITTO_LIBRARY} ${MOSQUITTOpp_LIBRARY})
#MESSAGE("MOSQUITTO_INCLUDE_DIR : ${MOSQUITTO_INCLUDE_DIR}")
#MESSAGE("MOSQUITTO_LIBRARIES : ${MOSQUITTO_LIBRARIES}")

include_directories(
  ${MOSQUITTO_INCLUDE_DIR}  
)

include_directories(include)

add_executable(mqtt_bridge_node
    src/mqtt_bridge_node.cpp
    #src/CMQTTmosquitto.cpp
    #include/mqtt_bridge/CMQTTmosquitto.h
)
ament_target_dependencies(mqtt_bridge_node rclcpp diagnostic_msgs)

#add_executable(mqtt_bridge_node2 src/mqtt_bridge_node2.cpp)
#ament_target_dependencies(mqtt_bridge_node2 rclcpp diagnostic_msgs)

#add_executable(CMQTTmosquitto src/CMQTTmosquitto.cpp)
#ament_target_dependencies(CMQTTmosquitto rclcpp diagnostic_msgs)

target_link_libraries(mqtt_bridge_node
  #${catkin_LIBRARIES}
  ${MOSQUITTO_LIBRARIES}
)

install(TARGETS
  mqtt_bridge_node
  #CMQTTmosquitto
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files and related config files.
install(
  #DIRECTORY include/
  #DESTINATION include
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(
  include
)

ament_package()
