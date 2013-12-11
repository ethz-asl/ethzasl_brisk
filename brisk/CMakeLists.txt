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

add_definitions(-msse2 -mssse3 -march=native -Wextra -Wall
                -Wno-overloaded-virtual -Wno-non-virtual-dtor # OpenCV.
                -std=c++11)

rosbuild_add_library(${PROJECT_NAME} src/brisk-descriptor-extractor.cc  
                                     src/brisk-feature-detector.cc   
                                     src/brisk-layer.cc
                                     src/brisk-scale-space.cc
                                     src/brute-force-matcher.cc   
                                     src/harris-feature-detector.cc
                                     src/harris-score-calculator.cc
                                     src/harris-score-calculator-float.cc
                                     src/harris-scores-sse.cc
                                     src/rdtsc-wrapper.cc
                                     src/sse-filters.cc
)

rosbuild_add_executable(demo src/demo.cc)
target_link_libraries(demo ${PROJECT_NAME})
