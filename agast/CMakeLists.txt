cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(ROS_BUILD_TYPE Release)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

file(GLOB AGAST_SOURCE_FILES  "${PROJECT_SOURCE_DIR}/src/*.cc")
file(GLOB AGAST_HEADER_FILES  "${PROJECT_SOURCE_DIR}/include/agast/*.h")

rosbuild_add_library(${PROJECT_NAME} ${AGAST_SOURCE_FILES})