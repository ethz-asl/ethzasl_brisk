cmake_minimum_required(VERSION 2.4.6)
project(brisk_ros_demo)

find_package(catkin REQUIRED COMPONENTS roscpp agast brisk image_transport)

catkin_package(
    DEPENDS opencv glog
    CATKIN_DEPENDS agast brisk image_transport roscpp
    INCLUDE_DIRS ${catkin_INCLUDE_DIRS}
    LIBRARIES
)
include_directories(${catkin_INCLUDE_DIRS})

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake/)

find_package(OpenCV REQUIRED)
find_package(GLog REQUIRED)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_definitions(-msse2 -mssse3 -march=native -Wextra -Wall
                -Wno-overloaded-virtual -Wno-non-virtual-dtor # OpenCV.
                -std=c++0x -DENABLE_BRISK_TIMING -DTEST_IN_SOURCE
                -DHAVE_OPENCV)

if (NOT (CMAKE_CXX_COMPILER_ID MATCHES "Clang"))
  add_definitions( -Werror )
endif()

add_executable(livedemo src/livedemo.cc)
target_link_libraries(livedemo ${catkin_LIBRARIES} 
                               ${GLOG_LIBRARY}
                               ${OpenCV_LIBS})