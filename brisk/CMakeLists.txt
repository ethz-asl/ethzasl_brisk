cmake_minimum_required(VERSION 2.4.6)
set(CMAKE_VERBOSE_MAKEFILE TRUE)

#include_directories("${OpenCV_INCLUDE_DIRS}")
#add_definitions(-I/usr/include)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#FIND_PACKAGE( OpenCV REQUIRED)
#include_directories(${OpenCV_INCLUDE_DIRS})
#add_definitions(${OpenCV_DEFINITIONS})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_definitions( -fPIC -msse2 -mssse3 -march=nocona -Wextra -Winit-self -Woverloaded-virtual -Wnon-virtual-dtor -Wsign-promo -Wno-long-long -std=c++0x)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} src/brisk.cpp include/brisk/hammingsse.hpp src/harris.cpp src/HarrisScoreCalculator.cpp src/HarrisScoreCalculatorFloat.cpp)
rosbuild_add_library(false_color src/false_color.cpp)
#target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)b
rosbuild_add_executable(livedemo src/livedemo.cpp)
target_link_libraries(livedemo ${PROJECT_NAME})

rosbuild_add_executable(benchdet src/bench_det.cpp)
target_link_libraries(benchdet ${PROJECT_NAME})

rosbuild_add_executable(flir_node src/flir.cpp)
target_link_libraries(flir_node ${PROJECT_NAME} false_color)

rosbuild_add_executable(openexrdemo src/OpenEXRimages_HighDynamicRange_Retina_toneMapping.cpp)

