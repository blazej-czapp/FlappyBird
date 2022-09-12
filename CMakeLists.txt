cmake_minimum_required(VERSION 3.13)
project(FlappyBird)
# find_package(PkgConfig REQUIRED)
find_package(X11)

set(CMAKE_CXX_STANDARD 17)

# like running pkg-config --libs opencv, puts all the results (without the -l prefixes) in OPENCV_LIBRARIES
# pkg_check_modules(OPENCV REQUIRED IMPORTED_TARGET opencv)
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} ${X11_INCLUDE_DIR})

include_directories(.)

add_compile_definitions(NDEBUG)

add_executable(FlappyBird
src/physicalArm.cpp
src/driver.cpp
src/display.cpp
src/main.cpp
src/featureDetector.cpp
src/util.hpp
src/units.hpp src/Recording.hpp src/Recording.cpp src/VideoSource.hpp src/WebCam.hpp)

target_compile_options(FlappyBird PRIVATE -O3)

# ${OPENCV_LIBRARIES} as found by pkg_check_modules()
target_link_libraries(FlappyBird k8055 usb pthread ${OpenCV_LIBS} ${X11_LIBRARIES})