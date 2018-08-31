SET(execName contourSmooth)
CMAKE_MINIMUM_REQUIRED(VERSION 2.4)
PROJECT(${execName})

aux_source_directory(. SRC_LIST)

find_package(OpenCV REQUIRED)
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

add_executable(${execName} ${SRC_LIST})

TARGET_LINK_LIBRARIES(${execName} ${OpenCV_LIBRARIES} )
