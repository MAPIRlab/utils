cmake_minimum_required(VERSION 3.8)
project(patrol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PatrolTimes.action"
)

include_directories(include)

# Add Executables
add_executable(patrol_times src/patrol_node.cpp)
ament_target_dependencies(patrol_times rclcpp rclcpp_action nav2_msgs geometry_msgs)

# Declare and install executables
install(TARGETS
  patrol_times
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Service/Action Dependencies
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)
target_link_libraries(patrol_times "${cpp_typesupport_target}")

ament_package()