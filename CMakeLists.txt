cmake_minimum_required(VERSION 3.17)
project(seam_carving)

set(CMAKE_CXX_STANDARD 11)

add_executable(seam_carving main.cpp SeamCarve.cpp SeamCarve.h)

find_package(OpenCV)

target_link_libraries(seam_carving ${OpenCV_LIBS})