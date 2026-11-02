cmake_minimum_required(VERSION 3.16)
project(cefiro_actuators VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=return-type)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cefiro_msgs REQUIRED)

add_library(cefiro_motor_model STATIC src/actuator_model.cpp)
target_include_directories(cefiro_motor_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(cefiro_actuators_node src/actuator_node.cpp)
target_link_libraries(cefiro_actuators_node cefiro_motor_model)
ament_target_dependencies(cefiro_actuators_node rclcpp std_msgs geometry_msgs cefiro_msgs)

install(TARGETS cefiro_actuators_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS cefiro_motor_model ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch docs DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/step_test.py scripts/fit_propeller_coefficients.py DESTINATION lib/${PROJECT_NAME})


ament_export_include_directories(include)
ament_export_libraries(cefiro_motor_model)
ament_package()
