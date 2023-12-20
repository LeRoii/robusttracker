cmake_minimum_required(VERSION 3.10)

project(yolov8)

add_definitions(-std=c++11)
add_definitions(-DAPI_EXPORTS)
set(CMAKE_CXX_STANDARD 11)
# set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/install)
set(CMAKE_SKIP_INSTALL_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  message("embed_platform on")
else()
  message("embed_platform off")

endif()

include_directories( ${CMAKE_SOURCE_DIR}/include)
#rga
include_directories(/usr/include/rga)


find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(MTTracking REQUIRED)

find_package(yaml-cpp REQUIRED)
find_package(spdlog REQUIRED)

set(APP_SRC 
  ${PROJECT_SOURCE_DIR}/demo.cpp
  ${PROJECT_SOURCE_DIR}/painter.cpp
  ${PROJECT_SOURCE_DIR}/serial.cpp 
  ${PROJECT_SOURCE_DIR}/camera.cpp  
  ${PROJECT_SOURCE_DIR}/realtracker.cpp
  )

add_executable(demo ${APP_SRC})

# target_link_libraries(demo sdireader  -lidetector  -ltracker ${OpenCV_LIBS})
target_link_libraries(demo ${OpenCV_LIBS} -ltracker ${RKNN_RT_LIB} ${RGA_LIB})
target_link_libraries(demo -lidetector)
target_link_libraries(demo yaml-cpp)
target_link_libraries(demo -pthread)
target_link_libraries(demo spdlog::spdlog)
