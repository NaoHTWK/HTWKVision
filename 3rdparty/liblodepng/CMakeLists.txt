cmake_minimum_required (VERSION 2.6)
project(lodepng)

include_directories(".")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -ffast-math -mfpmath=sse")

add_library(${PROJECT_NAME} lodepng.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
