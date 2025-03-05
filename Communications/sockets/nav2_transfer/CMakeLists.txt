cmake_minimum_required(VERSION 3.8)
project(nav2_transfer)

set(CMAKE_CXX_STANDARD 20)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(navToPoseClient src/navToPoseClient.cpp)
ament_auto_add_executable(navToPoseServer src/navToPoseServer.cpp)


install(
    TARGETS navToPoseClient navToPoseServer
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
