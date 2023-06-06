cmake_minimum_required(VERSION 3.23)

include_directories('/opt/homebrew/Cellar/eigen/3.4.0_1/include/eigen3')

project(useGeometry)

add_executable(useGeometry useGeometry.cpp)
add_executable(coordinateTransform coordinateTransform.cpp)
