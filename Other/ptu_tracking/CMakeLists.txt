cmake_minimum_required(VERSION 3.8)
project(ptu_tracking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)

# ptu d46
#find_package(ptu_interfaces REQUIRED)
# ptu interbotix
find_package(interbotix_xs_msgs)

find_package(Boost REQUIRED COMPONENTS system)

include_directories(include)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(ptu_tracking src/ptu_tracking.cpp)
ament_target_dependencies(ptu_tracking 
  rclcpp
  #ptu_interfaces 
  geometry_msgs 
  sensor_msgs 
  diagnostic_msgs 
  tf2_ros 
  tf2 
  std_msgs
  interbotix_xs_msgs
  Boost)

target_link_libraries(ptu_tracking
   ${Boost_LIBRARIES}
)

install(TARGETS
  ptu_tracking
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()