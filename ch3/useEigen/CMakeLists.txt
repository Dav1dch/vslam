cmake_minimum_required(VERSION 3.23)

project(useEigen)

include_directories("/opt/homebrew/Cellar/eigen/3.4.0_1/include/eigen3")

add_executable(eigenMatrix eigenMatrix.cpp)
