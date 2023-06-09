cmake_minimum_required(VERSION 3.23)
project(non_linear_optimization)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-std=c++14 -O3")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS},
	"/opt/homebrew/Cellar/eigen/3.4.0_1/include/eigen3")

add_executable(gaussNewton gaussNewton.cpp)

target_link_libraries(gaussNewton ${OpenCV_LIBS})
