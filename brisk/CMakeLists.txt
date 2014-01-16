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
#set(ROS_BUILD_TYPE Debug)

rosbuild_init()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

find_package(GLog)
include_directories(GLOG_INCLUDE_DIRS)
if(${GLOG_FOUND})
  add_definitions(-DHAVE_GLOG)
else()
  set(GLOG_LIBRARY "")
endif()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_definitions(-msse2 -mssse3 -march=native -Wextra -Wall
                -Wno-overloaded-virtual -Wno-non-virtual-dtor # OpenCV.
                -std=c++0x)

rosbuild_add_library(${PROJECT_NAME} src/brisk-descriptor-extractor.cc
                                     src/brisk-feature-detector.cc
                                     src/brisk-layer.cc
                                     src/brisk-scale-space.cc
                                     src/brute-force-matcher.cc
                                     src/camera-aware-feature.cc
                                     src/harris-feature-detector.cc
                                     src/harris-score-calculator.cc
                                     src/harris-score-calculator-float.cc
                                     src/harris-scores-sse.cc
                                     src/pattern-provider.cc
                                     src/sse-filters.cc
                                     src/timer.cc
                                     src/test/image-io.cc)
  
rosbuild_add_executable(demo src/demo.cc)
target_link_libraries(demo ${PROJECT_NAME})

rosbuild_add_executable(camera src/test-cameras.cc)
target_link_libraries(camera ${PROJECT_NAME})

rosbuild_add_library(test_libs src/test/serialization.cc
                               src/test/bench-ds.cc)

rosbuild_add_gtest(test_binary_equal src/test/test-binary-equal.cc)
target_link_libraries(test_binary_equal ${GLOG_LIBRARY} ${PROJECT_NAME} test_libs)


rosbuild_add_gtest(test_serialization src/test/test-serialization.cc)
target_link_libraries(test_serialization ${GLOG_LIBRARY} ${PROJECT_NAME} test_libs)
