cmake_minimum_required(VERSION 3.23)

project(useSophus)
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS},
	"/opt/homebrew/Cellar/eigen/3.4.0_1/include/eigen3")

add_executable(useSophus useSophus.cpp)
target_link_libraries(useSophus Sophus::Sophus)
