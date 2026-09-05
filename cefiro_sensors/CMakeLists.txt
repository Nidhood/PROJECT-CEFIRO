cmake_minimum_required(VERSION 3.8)

project(cefiro_sensors
    VERSION 1.0.0
    DESCRIPTION "Cefiro sensors package for configuration and test"
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # Enables ros2 & c++20 libraries to be detected by clangd.

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

install(DIRECTORY config launch urdf rviz
    DESTINATION share/${PROJECT_NAME}
)

ament_package()