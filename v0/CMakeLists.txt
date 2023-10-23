cmake_minimum_required(VERSION 3.10)

project(yolov8)

add_definitions(-std=c++11)
add_definitions(-DAPI_EXPORTS)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
enable_language(CUDA)

# include and link dirs of cuda and tensorrt, you need adapt them if yours are different
if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  message("embed_platform on")
  include_directories(/usr/local/cuda/targets/aarch64-linux/include)
  link_directories(/usr/local/cuda/targets/aarch64-linux/lib)
else()
  message("embed_platform off")
  # cuda
  include_directories(/usr/local/cuda/include)
  link_directories(/usr/local/cuda/lib64)

endif()

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(MTTracking REQUIRED)

find_package(yaml-cpp REQUIRED)
find_package(spdlog REQUIRED)

add_executable(demo ${PROJECT_SOURCE_DIR}/demo.cpp )
target_link_libraries(demo  -L${PROJECT_SOURCE_DIR} -lidetector -L${PROJECT_SOURCE_DIR} -ltracker ${OpenCV_LIBS})
target_link_libraries(demo MTTracking::mtracking MTTracking::mdetection yaml-cpp)
target_link_libraries(demo spdlog::spdlog)