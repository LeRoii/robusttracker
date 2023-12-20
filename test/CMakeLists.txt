cmake_minimum_required(VERSION 3.10)

project(yolov8)

add_definitions(-std=c++11)
add_definitions(-DAPI_EXPORTS)
set(CMAKE_CXX_STANDARD 11)
# set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE Debug)


find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories( ${CMAKE_SOURCE_DIR})

add_executable(demo main.cpp)

target_link_libraries(demo ${OpenCV_LIBS})