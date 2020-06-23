project(visual_slam)# Mandatory
cmake_minimum_required(VERSION 3.1)# Mandatory
set  (CMAKE_CXX_STANDARD 11)# Usec++11
message(STATUS "------- ${PROJECT_SOURCE_DIR}")
# tell cmake to output binaries here:
set  (CMAKE_ARCHIVE_OUTPUT_DIRECTORY${CMAKE_BINARY_DIR}/lib)
set  (CMAKE_LIBRARY_OUTPUT_DIRECTORY${CMAKE_BINARY_DIR}/lib)
set  (CMAKE_RUNTIME_OUTPUT_DIRECTORY${CMAKE_BINARY_DIR}/bin)
message(STATUS "-------executable path ${CMAKE_BINARY_DIR}")
# tell cmake where to look for*. h files
include_directories(${PROJECT_SOURCE_DIR}/src)
# create library "libtools"
add_library(tools src/tools.cpp)

find_package( OpenCV REQUIRED )
message(STATUS "----OPENCV-------- ${OpenCV_INCLUDE_DIRS}")
include_directories( ${OpenCV_INCLUDE_DIRS} )


# add executable main
add_executable(main src/sfm.cpp)
# tell the linker to bind these objects together
target_link_libraries(main tools ${OpenCV_LIBS})
