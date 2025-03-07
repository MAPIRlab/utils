cmake_minimum_required(VERSION 3.8)
project(tf_publisher_gui)

set(CMAKE_CXX_STANDARD 20)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(gui_pub src/gui_pub.cpp)

install(
    TARGETS gui_pub 
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
