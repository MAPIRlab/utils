cmake_minimum_required(VERSION 3.8)
project(ptu_tracking)


set(USE_GUI ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_marker_msgs)
# ptu interbotix
find_package(interbotix_xs_msgs)
find_package(Boost REQUIRED COMPONENTS system)

if(USE_GUI)
find_package(ament_imgui)
set(CONDITIONAL_IMGUI "ament_imgui") 
add_compile_definitions("USE_GUI=1")
endif()

include_directories(include)

include_directories(
  ${Boost_INCLUDE_DIRS}
)

add_executable(ptu_tracking src/ptu_tracking.cpp)
ament_target_dependencies(ptu_tracking 
  rclcpp
  image_marker_msgs
  geometry_msgs 
  sensor_msgs   
  tf2_ros 
  tf2 
  std_msgs
  interbotix_xs_msgs
  Boost
  ${CONDITIONAL_IMGUI} #if USE_GUI==OFF, the variable is empty
)

target_link_libraries(ptu_tracking
   ${Boost_LIBRARIES}
)

set_property(TARGET ptu_tracking PROPERTY CXX_STANDARD 20)

install(TARGETS
  ptu_tracking
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY resources
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()