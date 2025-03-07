cmake_minimum_required(VERSION 3.8)
project(tf_publisher_gui)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g")

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
