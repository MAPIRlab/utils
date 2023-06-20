cmake_minimum_required(VERSION 3.5)
project(coppelia_remote_api)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

set(COPPELIA_REMOTE_API_DIR "$ENV{COPPELIASIM_ROOT_DIR}/programming/zmqRemoteApi/clients/cpp")

add_subdirectory(${COPPELIA_REMOTE_API_DIR} ${CMAKE_BINARY_DIR}/zmqRemoteApi)

ament_export_include_directories(
    ${COPPELIA_REMOTE_API_DIR} 
    ${CMAKE_BINARY_DIR}/zmqRemoteApi/jsoncons/include
    )
ament_export_libraries(RemoteAPIClient)
ament_export_libraries(zmq)


install(
    TARGETS RemoteAPIClient
    DESTINATION lib
)
ament_package()