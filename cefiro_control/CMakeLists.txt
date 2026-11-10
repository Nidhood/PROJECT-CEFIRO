cmake_minimum_required(VERSION 3.8)

project(cefiro_control
    VERSION 1.1.0
    DESCRIPTION "CEFIRO's package for controllers logic"
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(
        -Wall
        -Wextra
        -Wpedantic
        -Werror=return-type
    )
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(cefiro_msgs REQUIRED)

# State feedback controller:
add_library(cefiro_attitude_controller SHARED
    src/cefiro_attitude_controller.cpp
)


target_include_directories(cefiro_attitude_controller PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(cefiro_attitude_controller
    controller_interface
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
    realtime_tools
    std_msgs
    cefiro_msgs
)

target_link_libraries(cefiro_attitude_controller
  Eigen3::Eigen
)

pluginlib_export_plugin_description_file(
    controller_interface
    cefiro_controllers_plugins.xml
)

install(
    TARGETS 
    
    cefiro_attitude_controller

    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
    DIRECTORY include/
    DESTINATION include
)

install(DIRECTORY config launch urdf
    DESTINATION share/${PROJECT_NAME}
)

# Independent simulation-only ROS 2 controller (GUI -> capture -> actuators).
install(PROGRAMS scripts/cefiro_balance_node.py
    DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/cefiro_multibody.py scripts/reference_envelope.py
    DESTINATION lib/${PROJECT_NAME})

ament_package()