cmake_minimum_required(VERSION 3.8)
project(gps2cartesian)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Geographiclib installs FindGeographicLib.cmake to this non-standard location
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED COMPONENTS STATIC)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Add Executables
add_executable(gps2cartesian src/gps2cartesian.cpp)
ament_target_dependencies(gps2cartesian 
  rclcpp
  geographic_msgs
  geometry_msgs 
  sensor_msgs 
  std_msgs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
)

# Declare and install executables
install(TARGETS
  gps2cartesian
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

target_link_libraries(gps2cartesian
  ${GeographicLib_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

ament_package()