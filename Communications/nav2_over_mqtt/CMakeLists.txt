cmake_minimum_required(VERSION 3.5)
project(nav2_over_mqtt)

##################
set(CMAKE_BUILD_TYPE "None") 
set(CMAKE_CXX_FLAGS "-std=c++17 -g -O0 ${CMAKE_CXX_FLAGS}")



find_package(ament_cmake REQUIRED)
set(AMENT_DEPS
    rclcpp
    rclcpp_action
    std_msgs
    nav2_msgs
    diagnostic_msgs
    spdlog
    )

foreach(Dependency IN ITEMS ${AMENT_DEPS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(include)

#client
add_executable(mqtt2Nav2 src/mqtt2Nav2.cpp)
ament_target_dependencies(mqtt2Nav2
${AMENT_DEPS}
) 
target_link_libraries(mqtt2Nav2 spdlog::spdlog)

#server
add_executable(nav2MqttSender src/nav2MqttSender.cpp)
ament_target_dependencies(nav2MqttSender
${AMENT_DEPS})
target_link_libraries(nav2MqttSender spdlog::spdlog) 


#test node
add_executable(testNode src/testNode.cpp)
ament_target_dependencies(testNode
${AMENT_DEPS}
) 

install(
  TARGETS mqtt2Nav2 nav2MqttSender testNode
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()