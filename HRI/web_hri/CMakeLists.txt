cmake_minimum_required(VERSION 3.5)
project(web_hri)

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
find_package(std_msgs REQUIRED)

include_directories(include)

# Add Executables
add_executable(webHRI src/web_hri.cpp)
ament_target_dependencies(webHRI rclcpp std_msgs)

# Declare and install executables
install(TARGETS
  webHRI
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(
  DIRECTORY launch web
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()