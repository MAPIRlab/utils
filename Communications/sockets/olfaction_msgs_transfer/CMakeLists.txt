cmake_minimum_required(VERSION 3.8)
project(olfaction_msgs_transfer)

set(CMAKE_CXX_STANDARD 20)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(anemometer_client src/anemometer/client.cpp)
ament_auto_add_executable(anemometer_server src/anemometer/server.cpp)

ament_auto_add_executable(gas_sensor_client src/gas_sensor/client.cpp)
ament_auto_add_executable(gas_sensor_server src/gas_sensor/server.cpp)


install(
    TARGETS anemometer_client anemometer_server gas_sensor_client gas_sensor_server
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
