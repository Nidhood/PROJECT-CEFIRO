cmake_minimum_required(VERSION 3.8)

project(cefiro_description
    VERSION 1.0.0
    DESCRIPTION "Cefiro description package"
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

install(DIRECTORY launch meshes urdf config rviz scripts
    DESTINATION share/${PROJECT_NAME} 
)

ament_package()