cmake_minimum_required(VERSION 3.8)
project(aruco)


set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-g -O3")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_marker_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)


include_directories(findAruco include)
add_executable(findAruco src/findAruco.cpp)
ament_target_dependencies( findAruco 
    rclcpp
    OpenCV 
    sensor_msgs
    image_marker_msgs
    message_filters
    tf2_ros
    tf2_geometry_msgs
    cv_bridge
)

add_executable(createMarker src/createMarker.cpp)
ament_target_dependencies( createMarker
    rclcpp
    OpenCV 
)


install(TARGETS
    createMarker findAruco
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)
ament_package()
