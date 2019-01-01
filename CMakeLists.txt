cmake_minimum_required(VERSION 3.13)
project(FlappyBird)
find_package(PkgConfig REQUIRED)

set(CMAKE_CXX_STANDARD 14)

pkg_check_modules(OPENCV REQUIRED opencv)

include_directories(.)

add_executable(FlappyBird
        src/arm.cpp
        src/bird.cpp
        src/camera.cpp
        src/main.cpp
        src/world.cpp src/util.hpp)

target_link_libraries(FlappyBird k8055 usb pthread ${OPENCV_LIBRARIES})