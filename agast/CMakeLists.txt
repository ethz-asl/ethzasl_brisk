cmake_minimum_required(VERSION 2.8.3)

#find_package(redwood_catkin)
if (redwood_catkin_FOUND)
  include(${PROJECT_SOURCE_DIR}/CMakeLists_catkin_simple.txt)
else()
  include(${PROJECT_SOURCE_DIR}/CMakeLists_rosbuild.txt)
endif()