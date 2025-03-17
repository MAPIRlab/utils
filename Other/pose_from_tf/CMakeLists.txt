cmake_minimum_required(VERSION 3.8)
project(pose_from_tf)

set(CMAKE_CXX_STANDARD 20)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(pose src/Pose.cpp)
ament_auto_add_executable(pose_with_covariance src/WithCovariance.cpp)

install(
    TARGETS pose pose_with_covariance
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
