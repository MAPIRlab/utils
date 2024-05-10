cmake_minimum_required(VERSION 3.15)
project(gps_viewer)

set(CMAKE_CXX_STANDARD 20)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS
    Core
    Gui
    Widgets
    Network
)

add_subdirectory(third_party/QGeoView)


include_directories(include)
add_executable(gps_viewer 
    src/gps_viewer_node.cpp 
    src/mainwindow.cpp 
    src/utils/placemarkcircle.cpp
    src/utils/helpers.cpp
    # QT headers must be included as sources
    include/gps_viewer/mainwindow.hpp
    include/gps_viewer/utils/placemarkcircle.hpp
)
ament_target_dependencies(gps_viewer
    rclcpp
)
target_link_libraries(gps_viewer
    QGeoView    
    Qt5::Core
    Qt5::Network
    Qt5::Gui
    Qt5::Widgets
)


install(TARGETS
    gps_viewer
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
