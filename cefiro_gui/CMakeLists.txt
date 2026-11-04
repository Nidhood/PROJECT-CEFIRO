cmake_minimum_required(VERSION 3.16)
project(cefiro_gui VERSION 1.5.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=return-type)
endif()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cefiro_msgs REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets Charts)
set(CMAKE_AUTOMOC OFF)
set(CMAKE_AUTORCC OFF)
set(CMAKE_AUTOUIC OFF)
qt6_add_resources(GUI_RESOURCES resources/resources.qrc)
add_executable(cefiro_gui_node
  src/main.cpp
  src/main_window.cpp
  src/ros_bridge.cpp
  src/scope_plot.cpp
  src/attitude_view.cpp
  src/attitude_command_pad.cpp
  src/recording_dialog.cpp
  ${GUI_RESOURCES}
)
target_include_directories(cefiro_gui_node PRIVATE include)
ament_target_dependencies(cefiro_gui_node rclcpp sensor_msgs geometry_msgs std_msgs cefiro_msgs)
target_link_libraries(cefiro_gui_node Qt6::Core Qt6::Gui Qt6::Widgets Qt6::Charts)
install(TARGETS cefiro_gui_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config resources DESTINATION share/${PROJECT_NAME})
ament_package()
