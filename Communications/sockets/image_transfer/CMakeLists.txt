cmake_minimum_required(VERSION 3.8)
project(image_transfer)

set(CMAKE_CXX_STANDARD 20)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(client_compressed_udp src/client_compressed_udp.cpp)
ament_auto_add_executable(server_compressed_udp src/server_compressed_udp.cpp)

ament_auto_add_executable(client_raw_udp src/client_raw_udp.cpp)
ament_auto_add_executable(server_raw_udp src/server_raw_udp.cpp)


install(
    TARGETS client_compressed_udp  server_compressed_udp  client_raw_udp  server_raw_udp
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)
ament_package()
