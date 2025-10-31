cmake_minimum_required(VERSION 3.8)
project(mapir_map_server)


find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(yaml-cpp)
ament_auto_find_build_dependencies()

ament_auto_add_executable(mapir_map_server src/mapir_map_server)
target_link_libraries(mapir_map_server yaml-cpp::yaml-cpp)
install(
    TARGETS mapir_map_server
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
