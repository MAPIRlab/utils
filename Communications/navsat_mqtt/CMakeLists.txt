cmake_minimum_required(VERSION 3.5)
project(navsat_mqtt)

##################
set(CMAKE_BUILD_TYPE "Release") 
set(CMAKE_CXX_FLAGS_RELEASE "-std=c++17 -g -O2 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "-std=c++17 -g -O0 ${CMAKE_CXX_FLAGS}")



find_package(ament_cmake REQUIRED)
set(AMENT_DEPS
    rclcpp
    std_msgs
    diagnostic_msgs
    sensor_msgs
    mqtt_serialization
    )

foreach(Dependency IN ITEMS ${AMENT_DEPS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(include)

#sender
add_executable(sender src/sender.cpp)
ament_target_dependencies(sender
${AMENT_DEPS}
) 

#receiver
add_executable(receiver src/receiver.cpp)
ament_target_dependencies(receiver
${AMENT_DEPS})


install(
  TARGETS sender receiver
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()