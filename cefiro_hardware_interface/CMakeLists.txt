cmake_minimum_required(VERSION 3.16)
project(cefiro_hardware_interface VERSION 0.3.0 LANGUAGES CXX)

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
find_package(std_srvs REQUIRED)
find_package(cefiro_msgs REQUIRED)

add_library(cefiro_hardware_core STATIC
  src/protocol.cpp
  src/serial_port.cpp
  src/motor_command.cpp
)
target_include_directories(cefiro_hardware_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(hardware_node src/hardware_node.cpp)
target_link_libraries(hardware_node cefiro_hardware_core)
ament_target_dependencies(hardware_node
  rclcpp sensor_msgs geometry_msgs std_msgs std_srvs cefiro_msgs
)

install(TARGETS hardware_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config docs DESTINATION share/${PROJECT_NAME})
install(FILES README.md LICENSE DESTINATION share/${PROJECT_NAME})

ament_package()
