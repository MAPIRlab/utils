cmake_minimum_required(VERSION 3.8)
project(image_transfer)

set(CMAKE_CXX_STANDARD 20)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(client_compressed src/compressed/client_compressed.cpp)
ament_auto_add_executable(server_compressed src/compressed/server_compressed.cpp)

ament_auto_add_executable(client_raw src/raw/client_raw.cpp)
ament_auto_add_executable(server_raw src/raw/server_raw.cpp)

ament_auto_add_executable(client_info src/cameraInfo/client.cpp)
ament_auto_add_executable(server_info src/cameraInfo/server.cpp)

install(
    TARGETS client_compressed  server_compressed  client_raw  server_raw client_info server_info
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)
ament_package()
