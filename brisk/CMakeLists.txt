cmake_minimum_required(VERSION 2.4.6)
set(CMAKE_VERBOSE_MAKEFILE TRUE)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_definitions(-fPIC -msse2 -mssse3 -march=nocona -Wextra -Winit-self 
                -Woverloaded-virtual -Wnon-virtual-dtor -Wsign-promo 
                -Wno-long-long)

rosbuild_add_library(${PROJECT_NAME} src/brisk.cpp
                                     src/harris.cpp 
                                     src/HarrisScoreCalculator.cpp 
                                     src/HarrisScoreCalculatorFloat.cpp 
                                     src/harrisScores_sse.cpp 
                                     src/rdtsc_wrapper.cpp)

