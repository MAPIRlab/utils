cmake_minimum_required(VERSION 3.8)
project(image_transfer)

set(CMAKE_CXX_STANDARD 20)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(client_compressed src/client_compressed.cpp)
ament_auto_add_executable(server_compressed src/server_compressed.cpp)

ament_auto_add_executable(client_raw src/client_raw.cpp)
ament_auto_add_executable(server_raw src/server_raw.cpp)


install(
    TARGETS client_compressed  server_compressed  client_raw  server_raw
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)
ament_package()
