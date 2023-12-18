cmake_minimum_required(VERSION 2.8)
project(test)

find_package(OpenCV REQUIRED)

if(NOT WIN32)
ADD_DEFINITIONS("-std=c++0x -O3")
endif(NOT WIN32)

include_directories(src) 
FILE(GLOB_RECURSE sourcefiles "src/*.cpp")
add_executable( KCF ${sourcefiles} )
target_link_libraries( KCF ${OpenCV_LIBS})


add_library(tracker SHARED ${PROJECT_SOURCE_DIR}/src/fhog.cpp
                           ${PROJECT_SOURCE_DIR}/src/kcftracker.cpp
                           ${PROJECT_SOURCE_DIR}/src/itracker.cpp
)
target_link_libraries( tracker ${OpenCV_LIBS})

add_custom_command(TARGET tracker POST_BUILD
                  COMMAND ${CMAKE_COMMAND} -E copy libtracker.so /usr/lib)


