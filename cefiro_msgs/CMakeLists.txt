cmake_minimum_required(VERSION 3.16)
project(cefiro_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorRpm.msg"
  "msg/MotorDshot.msg"
  "msg/MotorTelemetry.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
