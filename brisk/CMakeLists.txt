cmake_minimum_required(VERSION 2.8)
include(CheckIncludeFileCXX)

project(brisk)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions("-mssse3 -DHAVE_OPENCV")

include_directories(${CATKIN_DEVEL_PREFIX}/include)

check_include_file_cxx(opencv2/nonfree/nonfree.hpp HAVE_OPENCV_NONFREE_H)

add_definitions(-Wextra -Wall -pedantic)

if (NOT (CMAKE_CXX_COMPILER_ID MATCHES "Clang"))
  add_definitions( -Werror )
endif()

cs_add_library(${PROJECT_NAME} src/brisk-descriptor-extractor.cc
                               src/brisk-feature-detector.cc   
                               src/brisk-layer.cc
                               src/brisk-opencv.cc
                               src/brisk-scale-space.cc
                               src/brute-force-matcher.cc   
                               src/harris-feature-detector.cc
                               src/harris-score-calculator.cc
                               src/harris-score-calculator-float.cc
                               src/harris-scores.cc
                               src/image-down-sampling.cc
                               src/pattern-provider.cc
                               src/vectorized-filters.cc)  
                     
#if(HAVE_OPENCV_NONFREE_H)
# sleutenegger disabled check because that fails on ROS, 
# because it's located on some random path...
	cs_add_executable(demo src/demo.cc)
	target_link_libraries(demo ${PROJECT_NAME})
#endif()

cs_export(CFG_EXTRAS export_flags.cmake)
cs_install()
