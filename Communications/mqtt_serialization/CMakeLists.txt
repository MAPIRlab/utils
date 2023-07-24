cmake_minimum_required(VERSION 3.5)
project(mqtt_serialization)

find_package(ament_cmake REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

install(
    DIRECTORY include/mqtt_serialization
    DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(tf2_geometry_msgs)

ament_package()