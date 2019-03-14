cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)

add_library(basicFunc STATIC basicFunction.cpp basicFunction.h)
add_subdirectory(openpose)
target_link_libraries(basicFunc myOpenPose)