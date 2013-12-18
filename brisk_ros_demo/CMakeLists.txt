cmake_minimum_required(VERSION 2.4.6)
set(CMAKE_VERBOSE_MAKEFILE TRUE)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Release)
rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_definitions(-msse2 -mssse3 -march=native -Wextra -Wall -pedantic
                -Wno-overloaded-virtual -Wno-non-virtual-dtor #OpenCV
                )

rosbuild_add_executable(livedemo src/livedemo.cc)