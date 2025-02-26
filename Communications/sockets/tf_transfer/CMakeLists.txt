cmake_minimum_required(VERSION 3.8)
project(tf_transfer)

set(CMAKE_CXX_STANDARD 20)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(client src/client.cpp)
ament_auto_add_executable(server src/server.cpp)


install(
    TARGETS client server
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
