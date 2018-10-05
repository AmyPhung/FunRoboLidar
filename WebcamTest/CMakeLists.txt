cmake_minimum_required(VERSION 3.12)
project(WebcamTest)
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

set(CMAKE_CXX_STANDARD 14)

add_executable(WebcamTest main.cpp)
target_link_libraries( WebcamTest ${OpenCV_LIBS} )
