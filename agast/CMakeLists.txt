cmake_minimum_required(VERSION 2.8.3)
project(agast)

find_package(catkin_simple REQUIRED)
catkin_simple()

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
if(COMPILER_SUPPORTS_CXX14)
  add_definitions(-Wextra -Wall -pedantic -DHAVE_OPENCV -std=c++14)
else()
  message(FATAL_ERROR "The compiler does not support C++14.")
endif()

file(GLOB AGAST_SOURCE_FILES  "${PROJECT_SOURCE_DIR}/src/*.cc")
file(GLOB AGAST_HEADER_FILES  "${PROJECT_SOURCE_DIR}/include/agast/*.h")

cs_add_library(${PROJECT_NAME} ${AGAST_SOURCE_FILES})

cs_export(CFG_EXTRAS export_flags.cmake)
cs_install()
