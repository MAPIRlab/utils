cmake_minimum_required(VERSION 3.8)
project(mapir_map_server)


find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(mapir_map_server src/mapir_map_server)

install(
    TARGETS mapir_map_server
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
