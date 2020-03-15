cmake_minimum_required(VERSION 3.13)
project(FlappyBird)
find_package(PkgConfig REQUIRED)

set(CMAKE_CXX_STANDARD 17)

# like running pkg-config --libs opencv, puts all the results (without the -l prefixes) in OPENCV_LIBRARIES
pkg_check_modules(OPENCV REQUIRED opencv)

include_directories(.)

add_executable(FlappyBird
        src/arm.cpp
        src/driver.cpp
        src/display.cpp
        src/main.cpp
        src/featureDetector.cpp
        src/util.hpp
        src/units.hpp src/Recording.hpp src/Recording.cpp src/VideoSource.hpp src/WebCam.hpp)

# ${OPENCV_LIBRARIES} as found by pkg_check_modules()
target_link_libraries(FlappyBird k8055 usb pthread ${OPENCV_LIBRARIES})