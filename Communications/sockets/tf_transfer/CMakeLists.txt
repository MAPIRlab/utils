cmake_minimum_required(VERSION 3.8)
project(tf_transfer)

set(CMAKE_CXX_STANDARD 20)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(tf_client src/tf/client.cpp)
ament_auto_add_executable(tf_server src/tf/server.cpp)

ament_auto_add_executable(poseStamped_client src/poseStamped/client.cpp)
ament_auto_add_executable(poseStamped_server src/poseStamped/server.cpp)

ament_auto_add_executable(poseWithCovarianceStamped_client src/poseWithCovarianceStamped/client.cpp)
ament_auto_add_executable(poseWithCovarianceStamped_server src/poseWithCovarianceStamped/server.cpp)

install(
    TARGETS tf_client tf_server poseStamped_client poseStamped_server poseWithCovarianceStamped_client poseWithCovarianceStamped_server
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
