cmake_minimum_required(VERSION 2.8.3)
project(agast)

set(USE_CATKIN true CACHE BOOL "Whether to use catkin as a build system or not")

if (USE_CATKIN)

	find_package(catkin REQUIRED COMPONENTS roscpp)

	catkin_package(
	    DEPENDS
	    CATKIN_DEPENDS roscpp
	    INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
	    LIBRARIES agast
	    CFG_EXTRAS export_flags.cmake
	)
else()
	add_definitions(-DAGAST_GLOG)
endif()
find_package(OpenCV REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS})

if(APPLE)
  add_definitions(-Wextra -Wall -pedantic -DHAVE_OPENCV -std=c++11)
else()
  add_definitions(-Wextra -Wall -Werror -pedantic -DHAVE_OPENCV -std=c++0x)
endif()





file(GLOB AGAST_SOURCE_FILES  "${PROJECT_SOURCE_DIR}/src/*.cc")
file(GLOB AGAST_HEADER_FILES  "${PROJECT_SOURCE_DIR}/include/agast/*.h")

add_library(${PROJECT_NAME} ${AGAST_SOURCE_FILES})
