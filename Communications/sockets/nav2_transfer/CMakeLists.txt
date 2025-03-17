cmake_minimum_required(VERSION 3.8)
project(nav2_transfer)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g")

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(navToPoseClient src/navToPose/navToPoseClient.cpp)
ament_auto_add_executable(navToPoseServer src/navToPose/navToPoseServer.cpp)
ament_auto_add_executable(rvizTest src/navToPose/rvizTest.cpp)


ament_auto_add_executable(makePlanClient src/makePlan/client.cpp)
ament_auto_add_executable(makePlanServer src/makePlan/server.cpp)


ament_auto_add_executable(mapClient src/map/mapClient.cpp)
ament_auto_add_executable(mapServer src/map/mapServer.cpp)


ament_auto_add_executable(twistClient src/Twist/client.cpp)
ament_auto_add_executable(twistServer src/Twist/server.cpp)

install(
    TARGETS navToPoseClient navToPoseServer rvizTest mapClient mapServer twistClient twistServer makePlanClient makePlanServer
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
