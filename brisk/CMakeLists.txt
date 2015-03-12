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
                               src/camera-aware-feature.cc
                               src/harris-feature-detector.cc
                               src/harris-score-calculator.cc
                               src/harris-score-calculator-float.cc
                               src/harris-scores.cc
                               src/image-down-sampling.cc
                               src/pattern-provider.cc
                               src/vectorized-filters.cc
                               src/timer.cc
                               src/test/image-io.cc
                               src/brisk-v1.cc)  
                     
if(HAVE_OPENCV_NONFREE_H)
	cs_add_executable(demo src/demo.cc)
	target_link_libraries(demo ${PROJECT_NAME})
	
	cs_add_executable(mikolajczyk src/mikolajczyk.cc)
	target_link_libraries(mikolajczyk ${PROJECT_NAME})
endif()

cs_add_executable(camera src/test-cameras.cc)
target_link_libraries(camera ${PROJECT_NAME})

# Testing.
##########
  enable_testing()
  file(COPY src/test/test_data DESTINATION ${PROJECT_BINARY_DIR})

cs_add_library(${PROJECT_NAME}_test_lib src/test/serialization.cc
                                        src/test/bench-ds.cc
                                        src/opencv-ref.cc)
target_link_libraries(${PROJECT_NAME}_test_lib ${PROJECT_NAME})

catkin_add_gtest(test_integral_image src/test/test-integral-image.cc
                 WORKING_DIRECTORY ${PROJECT_BINARY_DIR})
target_link_libraries(test_integral_image ${GLOG_LIBRARY} 
                                          ${PROJECT_NAME}
                                          ${PROJECT_NAME}_test_lib)
                                          
catkin_add_gtest(test_popcnt src/test/test-popcount.cc
                 WORKING_DIRECTORY ${PROJECT_BINARY_DIR})
target_link_libraries(test_popcnt ${GLOG_LIBRARY} 
                                    ${PROJECT_NAME}
                                    ${PROJECT_NAME}_test_lib)

catkin_add_gtest(test_downsampling src/test/test-downsampling.cc
                 WORKING_DIRECTORY ${PROJECT_BINARY_DIR})
target_link_libraries(test_downsampling ${GLOG_LIBRARY} 
                                        ${PROJECT_NAME}
                                        ${PROJECT_NAME}_test_lib)

# TODO(slynen): The test files from Linux don't verify binary equal on OSX.
if(NOT APPLE)
  catkin_add_gtest(test_binary_equal src/test/test-binary-equal.cc
                   WORKING_DIRECTORY ${PROJECT_BINARY_DIR})
  target_link_libraries(test_binary_equal ${GLOG_LIBRARY} 
                                          ${PROJECT_NAME}
                                          ${PROJECT_NAME}_test_lib)
endif()

catkin_add_gtest(test_match src/test/test-match.cc
                 WORKING_DIRECTORY ${PROJECT_BINARY_DIR})
target_link_libraries(test_match ${GLOG_LIBRARY} 
                                   ${PROJECT_NAME}
                                   ${PROJECT_NAME}_test_lib)

catkin_add_gtest(test_serialization src/test/test-serialization.cc
                 WORKING_DIRECTORY ${PROJECT_BINARY_DIR})
target_link_libraries(test_serialization ${GLOG_LIBRARY} 
                                         ${PROJECT_NAME}
                                         ${PROJECT_NAME}_test_lib)

cs_export(CFG_EXTRAS export_flags.cmake)
cs_install()
