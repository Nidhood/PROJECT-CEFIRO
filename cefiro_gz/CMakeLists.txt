cmake_minimum_required(VERSION 3.8)

project(cefiro_gz
    VERSION 1.0.0
    DESCRIPTION "Cefiro Gazebo Spawner"
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
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_library(cefiro_rotor_system SHARED src/rotor_system.cpp)
target_link_libraries(cefiro_rotor_system PRIVATE
    gz-sim8::gz-sim8
    gz-plugin2::gz-plugin2
    gz-transport13::gz-transport13
    gz-msgs10::gz-msgs10
)
install(TARGETS cefiro_rotor_system LIBRARY DESTINATION lib)

install(DIRECTORY config launch models worlds scripts
    DESTINATION share/${PROJECT_NAME}
)

ament_package()