cmake_minimum_required (VERSION 2.6)
project(lodepng)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -ffast-math -mfpmath=sse -fPIC")

add_library(${PROJECT_NAME} lodepng.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC .)
