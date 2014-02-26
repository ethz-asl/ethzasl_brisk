cmake_minimum_required(VERSION 2.8)

project(brisk)

find_package(catkin REQUIRED COMPONENTS roscpp agast)
find_package(Eigen REQUIRED)

catkin_package(
    DEPENDS eigen glog gtest opencv
    CATKIN_DEPENDS roscpp agast
    INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
    LIBRARIES ${PROJECT_NAME}
    CFG_EXTRAS export_flags.cmake
)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake/)

find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GLog REQUIRED)

add_definitions("-mssse3 -DHAVE_OPENCV -std=c++0x")

include_directories(include  ${Eigen_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_definitions(-Wextra -Wall -Werror -pedantic)

add_library(${PROJECT_NAME} src/brisk-descriptor-extractor.cc
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
                            src/vectorized-filters.cc
                            src/timer.cc
                            src/test/image-io.cc)  
                     
target_link_libraries(${PROJECT_NAME} ${GLOG_LIBRARY} 
                                      ${OpenCV_LIBS}
                                      ${catkin_LIBRARIES})

add_executable(demo src/demo.cc)
target_link_libraries(demo ${PROJECT_NAME})

# Testing.
##########
if(CATKIN_ENABLE_TESTING AND NOT ANDROID)
  enable_testing()
  file(COPY src/test/test_data DESTINATION ${PROJECT_BINARY_DIR})

  add_library(${PROJECT_NAME}_test_lib src/test/serialization.cc
                                       src/test/bench-ds.cc
                                       src/opencv-ref.cc)

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

  catkin_add_gtest(test_binary_equal src/test/test-binary-equal.cc
                   WORKING_DIRECTORY ${PROJECT_BINARY_DIR})
  target_link_libraries(test_binary_equal ${GLOG_LIBRARY} 
                                          ${PROJECT_NAME}
                                          ${PROJECT_NAME}_test_lib)

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
                      
endif()